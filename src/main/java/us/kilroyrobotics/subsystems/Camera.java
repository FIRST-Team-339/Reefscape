package us.kilroyrobotics.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import us.kilroyrobotics.Constants.CameraConstants;

public class Camera extends SubsystemBase {
    private UsbCamera camera;
    private VideoSink server;
    private VideoSource source;

    /* Constructor */
    public Camera() {
        if (CameraConstants.kCameraEnabled) {
            camera = CameraServer.startAutomaticCapture("Camera", 0);
            server = CameraServer.getServer("serve_Camera");
            setCameraValues(camera, server);
        }
    }

    /**
     * The {@code setCameraValues} method is to update the camera values such as:
     *
     * <ul>
     *   <li>Resolution
     *   <li>Frames per Second (FPS)
     *   <li>Compression rate
     *   <li>Brightness
     * </ul>
     *
     * The values can all be changed and set under {@link CameraConstants}
     *
     * @param usbCamera The {@link UsbCamera} to set the values for
     * @param server The camera server ({@link VideoSink}) to update the value(s) for
     */
    public void setCameraValues(UsbCamera usbCamera, VideoSink server) {
        usbCamera.setResolution(CameraConstants.kResolution[0], CameraConstants.kResolution[1]);
        usbCamera.setFPS(CameraConstants.kFPS);
        server.getProperty("compression").set(CameraConstants.kCompression);
        usbCamera.setBrightness(CameraConstants.kBrightness);

        source = server.getSource();
    }

    public UsbCamera getCamera() {
        return camera;
    }

    public VideoSource getVideoSource() {
        return source;
    }
}
