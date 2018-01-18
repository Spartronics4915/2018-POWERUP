package com.spartronics4915.frc2018;

import java.io.File;
import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.StandardWatchEventKinds;
import java.nio.file.WatchEvent;
import java.nio.file.WatchKey;
import java.nio.file.WatchService;

import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Controls the first person view stream, ensuring that the service will be
 * restarted and if the webcam is unplugged and
 * changes names.
 */
public class VideoStreamServiceController
{

    private Process mExistingProcess;

    public void registerWatcher()
    {
        Logger.notice("VideoStreamServiceController registering watcher");
        FileSystem fileSystem = FileSystems.getDefault();

        Path devPath = fileSystem.getPath("/dev");
        try
        {
            final WatchService watchService = fileSystem.newWatchService();
            devPath.register(watchService, StandardWatchEventKinds.ENTRY_CREATE);
            Logger.notice("VideoStreamServiceController watcher registered");

            new Thread(() ->
            {
                findInitialVideoDevice();
                for (;;)
                {
                    try
                    {
                        Logger.debug("VideoStreamServiceController waiting for key");
                        WatchKey key = watchService.take();
                        Logger.notice("got key");
                        for (WatchEvent<?> event : key.pollEvents())
                        {
                            WatchEvent.Kind<?> kind = event.kind();
                            if (kind != StandardWatchEventKinds.ENTRY_CREATE)
                            {
                                Logger.notice("File watcher Non create event");
                                continue;
                            }

                            Path newFilename = devPath.resolve(((WatchEvent<Path>) event).context());
                            if (!newFilename.getFileName().toString().startsWith("video"))
                            {
                                Logger.debug(
                                        "VideoStreamServiceController Don't care about new device: " + newFilename.toString());
                                continue;
                            }

                            Logger.debug("VideoStreamServiceController New video device: " + newFilename.toString());
                            restartProcess(newFilename);
                        }
                        key.reset();
                    }
                    catch (InterruptedException e)
                    {
                        e.printStackTrace();
                        Logger.notice("Exception: " + e);
                        continue;
                    }
                }
            }).run();
        }
        catch (IOException e)
        {
            e.printStackTrace();
            DriverStation.reportError("Couldn't start video service: " + e.getMessage(), false);
        }

    }

    private void findInitialVideoDevice()
    {
        File devFolder = new File("/dev");
        File[] videoDevices = devFolder.listFiles((dir, name) -> name.startsWith("video"));
        if (videoDevices.length == 0)
        {
            Logger.notice("VideoStreamServiceController no video devices found");
        }
        else
        {
            Logger.notice("VideoStreamServiceController using video device " + videoDevices[0].toString());
            restartProcess(videoDevices[0].toPath());
        }
    }

    private void restartProcess(Path newVideoDevice)
    {
        Logger.notice("Is old process alive? " + (mExistingProcess != null && mExistingProcess.isAlive()));
        if (mExistingProcess != null)
        {
            mExistingProcess.destroy();
            try
            {
                Logger.debug("Wait for old process");
                mExistingProcess.waitFor();
                Logger.debug("Done waiting for old process");
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            mExistingProcess = null;
        }

        try
        {
            mExistingProcess = new ProcessBuilder(
                    "/usr/local/bin/mjpg_streamer",
                    "-i",
                    "/usr/local/lib/mjpg-streamer/input_uvc.so --device " + newVideoDevice.toString() + " -r 160x120",
                    "-o",
                    "/usr/local/lib/mjpg-streamer/output_http.so -w /usr/local/share/mjpg-streamer/www -p 5801")
                            .redirectError(ProcessBuilder.Redirect.INHERIT)
                            .redirectOutput(ProcessBuilder.Redirect.INHERIT)
                            .start();
            Logger.notice("New process alive: " + mExistingProcess.isAlive());
        }
        catch (IOException e)
        {
            DriverStation.reportError("Didn't start mjpg-streamer: " + e.getMessage(), false);
            e.printStackTrace();
        }
    }
}
