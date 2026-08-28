#pragma once

#include "CoreMinimal.h"
#include "ROS2Publisher.h"

#include "BlueSimCameraPublisher.generated.h"

class UTextureRenderTarget2D;
class USceneCaptureComponent2D;
class UROS2GenericMsg;

UCLASS(Blueprintable, BlueprintType)
class BLUESIM_API UBlueSimCameraPublisher : public UROS2Publisher
{
    GENERATED_BODY()

public:

    /**
     * Render target containing the camera image.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BlueSim Camera")
    UTextureRenderTarget2D* RenderTarget = nullptr;

    /**
     * Optional Scene Capture Component.
     *
     * If assigned, the publisher captures a fresh scene immediately
     * before reading the render target.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BlueSim Camera")
    USceneCaptureComponent2D* CaptureComponent = nullptr;

    /**
     * Flip the image vertically after reading the render target.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "BlueSim Camera")
    bool bFlipVertical = true;

protected:

    virtual void InitializeTopicComponent() override;

public:

    virtual void UpdateMessage(UROS2GenericMsg* InMessage) override;
};
