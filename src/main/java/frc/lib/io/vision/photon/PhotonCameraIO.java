package frc.lib.io.vision.photon;

import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.io.vision.CameraIO;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCameraIO extends CameraIO {

  private final PhotonCamera wrappedCamera;
  protected List<PhotonPipelineResult> lastInputBuffer = new ArrayList<>();

  public PhotonCameraIO(CameraIOConfig config) {
    super(config);
    wrappedCamera = new PhotonCamera(config.name);
  }

  @Override
  public void update() {
    lastInputBuffer = wrappedCamera.getAllUnreadResults();
  }

  @Override
  public boolean isConnected() {
    return wrappedCamera.isConnected();
  }

  public List<PhotonPipelineResult> getLastInputBuffer() {
    return lastInputBuffer;
  }
}
