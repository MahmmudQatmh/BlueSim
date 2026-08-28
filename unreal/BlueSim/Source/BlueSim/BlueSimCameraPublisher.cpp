#include "BlueSimCameraPublisher.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Msgs/ROS2Img.h"

#include "RenderingThread.h"
#include "RHI.h"

void UBlueSimCameraPublisher::InitializeTopicComponent()
{
    /*
     * We deliberately use the overridden UpdateMessage() as the
     * publisher callback.
     *
     * ROS2PublisherComponent normally takes its Update Delegate
     * from Blueprint. Here we provide the update implementation
     * directly in this custom publisher.
     */
    SetDefaultDelegates();

    Super::InitializeTopicComponent();
}

void UBlueSimCameraPublisher::UpdateMessage(UROS2GenericMsg* InMessage)
{
    if (!IsValid(RenderTarget))
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[BlueSimCameraPublisher] RenderTarget is not assigned.")
        );

        return;
    }

    UROS2ImgMsg* ImageMessage = Cast<UROS2ImgMsg>(InMessage);

    if (!IsValid(ImageMessage))
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[BlueSimCameraPublisher] Message is not ROS2ImgMsg.")
        );

        return;
    }

    /*
     * Capture a fresh image immediately before reading the render target.
     *
     * This means the ROS publication frequency controls the camera
     * acquisition frequency.
     */
    if (IsValid(CaptureComponent))
    {
        CaptureComponent->CaptureScene();
    }

    const int32 Width = RenderTarget->SizeX;
    const int32 Height = RenderTarget->SizeY;

    if (Width <= 0 || Height <= 0)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[BlueSimCameraPublisher] Invalid RenderTarget size: %d x %d"),
            Width,
            Height
        );

        return;
    }

    FTextureRenderTargetResource* RenderTargetResource =
        RenderTarget->GameThread_GetRenderTargetResource();

    if (RenderTargetResource == nullptr)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[BlueSimCameraPublisher] Failed to get RenderTarget resource.")
        );

        return;
    }

    TArray<FColor> Pixels;

    FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
    ReadFlags.SetLinearToGamma(true);

    if (!RenderTargetResource->ReadPixels(Pixels, ReadFlags))
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT("[BlueSimCameraPublisher] ReadPixels failed.")
        );

        return;
    }

    const int32 ExpectedPixelCount = Width * Height;

    if (Pixels.Num() != ExpectedPixelCount)
    {
        UE_LOG(
            LogTemp,
            Error,
            TEXT(
                "[BlueSimCameraPublisher] Invalid pixel count. "
                "Expected=%d Received=%d"
            ),
            ExpectedPixelCount,
            Pixels.Num()
        );

        return;
    }

    /*
     * ROS encoding:
     *
     * bgr8 = 3 bytes per pixel
     *
     * Unreal FColor stores:
     *
     *   B
     *   G
     *   R
     *   A
     *
     * Therefore we can directly copy B/G/R and ignore A.
     */
     FROSImg ImageData;

     // ROS frame associated with this camera.
     ImageData.Header.FrameId = TEXT("boat_camera");

     ImageData.Width = Width;
     ImageData.Height = Height;
     ImageData.Encoding = TEXT("bgr8");
     ImageData.IsBigendian = 0;
     ImageData.Step = Width * 3;

    ImageData.Data.SetNumUninitialized(
        Width * Height * 3
    );

    for (int32 Y = 0; Y < Height; ++Y)
    {
        const int32 SourceY =
            bFlipVertical
                ? (Height - 1 - Y)
                : Y;

        for (int32 X = 0; X < Width; ++X)
        {
            const int32 SourceIndex =
                SourceY * Width + X;

            const int32 DestinationIndex =
                (Y * Width + X) * 3;

            const FColor& Pixel =
                Pixels[SourceIndex];

            ImageData.Data[DestinationIndex + 0] = Pixel.B;
            ImageData.Data[DestinationIndex + 1] = Pixel.G;
            ImageData.Data[DestinationIndex + 2] = Pixel.R;
        }
    }

    /*
     * Copy the populated FROSImg into the actual
     * sensor_msgs/msg/Image message owned by rclUE.
     */
    ImageMessage->SetMsg(ImageData);
}
