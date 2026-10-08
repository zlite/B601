"""Build the optional read-only feedback extension; never opens robot devices."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
COMMIT = 'c652ce420e7da1008fa1864cc7f47d7d7c57fdb6'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cargo', default='cargo', help='Rust 1.90.0 cargo executable')
    args = parser.parse_args()
    def run(*command, cwd):
        return subprocess.run(command, cwd=cwd, check=True, text=True)
    with tempfile.TemporaryDirectory(prefix='b601-motorbridge-') as temporary:
        source = Path(temporary)/'source'
        run('git', 'clone', '--depth', '1', '--branch', 'v0.5.6',
            'https://github.com/motorbridge/motorbridge.git', str(source), cwd=temporary)
        actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source, text=True).strip()
        if actual != COMMIT:
            raise RuntimeError('Upstream tag no longer matches the reviewed commit')
        run('git', 'apply', str(HERE/'fresh_state.patch'), cwd=source)
        run(args.cargo, 'test', '--locked', '-p', 'motor_vendor_damiao', '--lib', cwd=source)
        run(args.cargo, 'build', '--locked', '-p', 'motor_abi', '--release', cwd=source)
        destination = HERE/'build'
        destination.mkdir(exist_ok=True)
        library = destination/'libmotor_abi.so'
        shutil.copyfile(source/'target/release/libmotor_abi.so', library)
        manifest = {'upstream_commit': COMMIT,
                    'patch_sha256': hashlib.sha256((HERE/'fresh_state.patch').read_bytes()).hexdigest(),
                    'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest()}
        (destination/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print(library)


if __name__ == '__main__':
    main()
