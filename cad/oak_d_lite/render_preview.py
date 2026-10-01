from pathlib import Path
import vtk
P=Path(__file__).resolve().parent
win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(1500,650);win.SetMultiSamples(8)

def actor(path,color):
 reader=vtk.vtkSTLReader();reader.SetFileName(str(path));reader.Update()
 mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(reader.GetOutputPort())
 a=vtk.vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(*color);a.GetProperty().SetAmbient(.23);a.GetProperty().SetDiffuse(.75);return a
for i in range(2):
 r=vtk.vtkRenderer();r.SetViewport(i/2,0,(i+1)/2,1);r.SetBackground(.95,.96,.98);win.AddRenderer(r)
 r.AddActor(actor(P/'OAK_D_Lite_B601_v2_assembly_coordinates.stl',(.22,.64,.47)))
 if i:
  r.AddActor(actor(P/'camera_positioned.stl',(.16,.18,.21)))
  # Lens aperture markers derived from official enclosure centers; preview only.
  for x in (-37.5,0,37.5):
   disk=vtk.vtkDiskSource();disk.SetInnerRadius(0);disk.SetOuterRadius(3.5 if x else 4.0);disk.SetCircumferentialResolution(64);disk.Update()
   mapper=vtk.vtkPolyDataMapper();mapper.SetInputConnection(disk.GetOutputPort());a=vtk.vtkActor();a.SetMapper(mapper);a.SetPosition(x,5.539,48.47);a.GetProperty().SetColor(.25,.4,.5);r.AddActor(a)
 cam=r.GetActiveCamera();cam.SetPosition(110,-140,130);cam.SetFocalPoint(0,0,24);cam.SetViewUp(0,1,0);cam.ParallelProjectionOn();r.ResetCamera();cam.Zoom(1.16)
 title=vtk.vtkTextActor();title.SetInput(['PRINTED MOUNT','WITH OAK-D LITE'][i]);title.SetDisplayPosition(30,602);title.GetTextProperty().SetFontSize(23);title.GetTextProperty().SetColor(.13,.19,.24);r.AddActor2D(title)
 caption=vtk.vtkTextActor();caption.SetInput(['Original arm attachment | two M4 camera screws','Official enclosure CAD | lenses shown schematically'][i]);caption.SetDisplayPosition(30,24);caption.GetTextProperty().SetFontSize(16);caption.GetTextProperty().SetColor(.25,.3,.35);r.AddActor2D(caption)
win.Render();grab=vtk.vtkWindowToImageFilter();grab.SetInput(win);grab.Update();png=vtk.vtkPNGWriter();png.SetFileName(str(P/'OAK_D_Lite_B601_v2_preview.png'));png.SetInputConnection(grab.GetOutputPort());png.Write()
