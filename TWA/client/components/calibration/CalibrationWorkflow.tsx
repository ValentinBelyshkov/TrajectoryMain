import React from "react";
import { CalibrationTypeSelection } from "./CalibrationTypeSelection";
import { AutoCalibrationRegion } from "./AutoCalibrationRegion";
import { AutoCalibrationImageSelect } from "./AutoCalibrationImageSelect";
import { InstructionsStep } from "./InstructionsStep";
import { ImageUploadStep } from "./ImageUploadStep";
import { TestRunStep } from "./TestRunStep";
import { FrameSelectionStep } from "./FrameSelectionStep";
import { CameraRecordingStep } from "./CameraRecordingStep";
import { FrameTrimmingStep } from "./FrameTrimmingStep";
import { VideoUploadStep } from "./VideoUploadStep";
import { ProcessingStep } from "./ProcessingStep";
import { CorrelationStep } from "./CorrelationStep";
import { FinalizeStep } from "./FinalizeStep";
import { CalibrationPointSelector } from "@/components/CalibrationPointSelector";
import { useCalibration } from "@/contexts/CalibrationContext";

interface CalibrationWorkflowProps {
  onStartPublisher?: (projectId: string) => Promise<void>;
}

export function CalibrationWorkflow({
  onStartPublisher,
}: CalibrationWorkflowProps) {
  const c = useCalibration();

  const calibrationStep = c.calibrationStep;
  const projectId = c.project?.id;
  const projectType = c.project?.type;
  const publisherMode = c.systemStatus?.publisher_mode;
  const projectVideoFilename = c.project?.videoFilename;

  const goTo = c.setCalibrationStep;

  const wrapOverlay = (content: React.ReactNode) => (
    <div className="fixed inset-0 z-[1200] bg-slate-50 flex flex-col">
      <div className="h-full w-full bg-white shadow-xl relative overflow-hidden flex flex-col">
        <button
          onClick={c.handleCalibrationCancel}
          className="absolute top-4 right-4 text-muted-foreground hover:text-foreground z-10 p-2 hover:bg-slate-100 rounded-full transition-colors"
        >
          <span className="text-xl">✕</span>
        </button>
        <div className="flex-1 min-h-0">{content}</div>
      </div>
    </div>
  );

  // Type selection step
  if (calibrationStep === "type-selection") {
    return wrapOverlay(
      <CalibrationTypeSelection
        onSelect={c.handleCalibrationTypeSelect}
        onBack={c.handleCalibrationCancel}
      />,
    );
  }

  // Auto calibration steps
  if (calibrationStep === "auto-region") {
    return wrapOverlay(
      <AutoCalibrationRegion
        onConfirm={c.handleAutoRegionConfirm}
        onBack={c.handleAutoCalibrationBack}
        dronePosition={c.dronePosition}
        error={c.autoCalibrationError}
        progress={c.autoCalibrationProgress}
        message={c.autoCalibrationMessage}
      />,
    );
  }

  if (calibrationStep === "auto-image-select") {
    return wrapOverlay(
      <AutoCalibrationImageSelect
        frames={c.autoCalibrationFrames}
        onSelect={c.handleAutoImageSelect}
        onBack={c.handleAutoCalibrationBack}
        error={c.autoCalibrationError}
        progress={c.autoCalibrationProgress}
        message={c.autoCalibrationMessage}
      />,
    );
  }

  if (calibrationStep === "instructions") {
    return wrapOverlay(
      <div className="flex flex-col h-full">
        <div className="flex-1 flex items-center justify-center">
          <InstructionsStep onNext={() => {}} />
        </div>
        <div className="p-6 border-t bg-slate-50 flex justify-center shrink-0">
          <button
            onClick={c.handleInstructionsNext}
            className="w-full max-w-md bg-primary text-white py-3 rounded-lg font-bold flex items-center justify-center gap-2 shadow-lg shadow-primary/20"
          >
            <span className="w-4 h-4">▶</span>
            Продолжить
          </button>
        </div>
      </div>,
    );
  }

  if (calibrationStep === "test-run" && projectId) {
    return wrapOverlay(
      <TestRunStep
        projectId={projectId}
        onSuccess={c.handleTestRunSuccess}
        onBack={c.handleTestRunBack}
        hasVideoStream={c.hasVideoStream}
        videoCanvasRef={c.videoCanvasRef}
      />,
    );
  }

  if (calibrationStep === "frame-selection" && projectId) {
    return wrapOverlay(
      <FrameSelectionStep
        projectId={projectId}
        onFramesSelected={c.handleFramesSelected}
        onBack={c.handleFrameSelectionBack}
      />,
    );
  }

  if (calibrationStep === "upload") {
    return wrapOverlay(
      <ImageUploadStep
        onUpload={c.handleImageUpload}
        uploadError={c.uploadError}
        isUploading={c.isUploading}
        onErrorDismiss={c.clearUploadError}
        onFileSelect={() => {}}
      />,
    );
  }

  if (calibrationStep === "pairing") {
    if (c.selectedFrames.length > 0) {
      return (
        <CalibrationPointSelector
          images={c.selectedFrames}
          onComplete={c.handleCalibrationComplete}
          onCancel={c.handleCalibrationCancel}
          projectId={projectId}
        />
      );
    }
    if (c.uploadedImage) {
      return (
        <CalibrationPointSelector
          imageUrl={c.uploadedImage.url}
          onComplete={c.handleCalibrationComplete}
          onCancel={c.handleCalibrationCancel}
          projectId={projectId}
          imageFilename={c.uploadedImage.filename}
        />
      );
    }
  }

  if (calibrationStep === "recording") {
    const isCamera =
      projectType === "камера" || publisherMode === "realsense";
    if (!isCamera) {
      return wrapOverlay(
        <VideoUploadStep
          onComplete={goTo.bind(null, "trimming") as () => void}
          onBack={c.handleCalibrationCancel}
          onSkip={goTo.bind(null, "trimming") as () => void}
          sessionId={c.calibrationSessionId}
          onSessionIdChange={c.setCalibrationSessionId}
          onRecordingStatusChange={c.setRecordingStatus}
          onRecordingDurationChange={c.setRecordingDuration}
          projectId={projectId}
          projectVideoFilename={projectVideoFilename}
        />,
      );
    }
    return wrapOverlay(
      <CameraRecordingStep
        onStart={c.handleStartRecording}
        onStop={c.handleStopRecording}
        videoCanvasRef={c.videoCanvasRef}
        hasVideoStream={c.hasVideoStream}
        calibrationSessionId={c.calibrationSessionId}
        sessionStatus={c.calibrationSession?.status}
        startSession={c.startNewCalibrationSession}
        onComplete={goTo.bind(null, "processing") as () => void}
        onBack={c.handleCalibrationCancel}
        onSkip={async () => {
          if (!c.calibrationSessionId && projectId) {
            try {
              // Не очищаем папку frames: при пропуске записи кадры уже есть
              // на диске и их нельзя удалять.
              await c.startNewCalibrationSession(false);
            } catch (err) {
              console.error("Failed to start calibration session on skip:", err);
            }
          }
          goTo("processing");
        }}
        error={c.recordingError}
        projectId={projectId}
        onStartPublisher={onStartPublisher}
        setCalibrationSessionId={c.setCalibrationSessionId}
      />,
    );
  }

  if (calibrationStep === "trimming") {
    // Both camera (RealSense) and simulation recordings are processed
    // frame-by-frame: frames live in the project's frames folder, and the
    // trimming editor deletes the extra frames in place (no video/calib
    // creation on the backend). Use the same frame-based editor for both.
    return wrapOverlay(
      <FrameTrimmingStep
        projectId={projectId}
        onApplyTrim={c.handleApplyFrameTrim}
        onBack={goTo.bind(null, "recording") as () => void}
        onSkip={goTo.bind(null, "processing") as () => void}
      />,
    );
  }

  if (calibrationStep === "processing") {
    return wrapOverlay(
      <ProcessingStep
        sessionId={c.calibrationSessionId || ""}
        progress={c.processingProgress}
        onComplete={c.handleRunProcessing}
        onBack={goTo.bind(null, "trimming") as () => void}
        onSkip={c.handleSkipProcessing}
      />,
    );
  }

  if (calibrationStep === "correlating") {
    return wrapOverlay(
      <CorrelationStep
        sessionId={c.calibrationSessionId || ""}
        projectId={projectId || ""}
        frames={c.calibrationSession?.frame_pose_data || []}
        onCompute={c.handleComputeCorrelation}
        onBack={goTo.bind(null, "processing") as () => void}
        sessionStatus={c.calibrationSession?.status}
      />,
    );
  }

  if (calibrationStep === "finalizing") {
    return wrapOverlay(
      <FinalizeStep
        sessionId={c.calibrationSessionId || ""}
        transform={c.transform}
        onComplete={c.handleFinalizeCalibration}
        onBack={goTo.bind(null, "correlating") as () => void}
        onSkip={goTo.bind(null, "complete") as () => void}
      />,
    );
  }

  return null;
}
