import type { CalibrationStep } from "@/hooks/useProject";
import React from "react";
import { CalibrationTypeSelection } from "./CalibrationTypeSelection";
import { AutoCalibrationRegion } from "./AutoCalibrationRegion";
import { AutoCalibrationImageSelect } from "./AutoCalibrationImageSelect";
import { InstructionsStep } from "./InstructionsStep";
import { ImageUploadStep } from "./ImageUploadStep";
import { TestRunStep } from "./TestRunStep";
import { FrameSelectionStep } from "./FrameSelectionStep";
import { VideoRecordingStep } from "./VideoRecordingStep";
import { VideoTrimmingStep } from "./VideoTrimmingStep";
import { VideoUploadStep } from "./VideoUploadStep";
import { ProcessingStep } from "./ProcessingStep";
import { CorrelationStep } from "./CorrelationStep";
import { FinalizeStep } from "./FinalizeStep";
import {
  CalibrationPointSelector,
  type CalibrationPoint,
} from "@/components/CalibrationPointSelector";
import type { AutoCalibrationRegion as AutoCalibrationRegionType } from "@/lib/api";
import type { RefObject } from "react";

interface CalibrationWorkflowProps {
  calibrationStep: CalibrationStep;
  uploadedImage: { filename: string; url: string } | null;
  selectedFrames: { filename: string; url: string }[];
  uploadError: string | null;
  isUploading: boolean;
  onTypeSelect: (type: "manual" | "auto") => void;
  onInstructionsNext: () => void;
  onTestRunSuccess: () => void;
  onTestRunBack: () => void;
  onFrameSelectionBack: () => void;
  onFramesSelected: (frames: { filename: string; url: string }[]) => void;
  onImageUpload: (file: File) => void;
  onCalibrationComplete: () => void;
  onCalibrationCancel: () => void;
  onUploadErrorDismiss: () => void;
  projectId?: string;
  projectType?: "камера" | "симуляция";
  publisherMode?: string;
  projectVideoFilename?: string | null;
  hasVideoStream: boolean;
  videoCanvasRef: RefObject<HTMLCanvasElement | null>;
  // Auto calibration props
  dronePosition: { lat: number; lng: number };
  autoCalibrationRegion: AutoCalibrationRegionType | null;
  autoCalibrationFrames: { filename: string; url: string }[];
  autoCalibrationError: string | null;
  autoCalibrationProgress: "idle" | "downloading" | "matching" | "success" | "error";
  autoCalibrationMessage: string | null;
  onAutoRegionConfirm: (region: AutoCalibrationRegionType) => void;
  onAutoImageSelect: (imageFilename: string) => void;
  onAutoCalibrationBack: () => void;
  // NEW manual calibration props
  calibrationSessionId: string | null;
  onSessionIdChange: (id: string) => void;
  calibrationSession: any;
  recordingStatus: "idle" | "recording" | "stopped";
  recordingDuration: number;
  onRecordingDurationChange: (duration: number) => void;
  onRecordingStatusChange: (status: "idle" | "recording" | "stopped") => void;
  trimSegments: Array<{ start: number; end: number }>;
  processingProgress: string;
  correlationPoints: any[];
  transform: any;
  onStartRecording: () => void;
  onStopRecording: () => void;
  onApplyTrim: (segments: Array<{ start: number; end: number }>) => void;
  onRunProcessing: () => void;
  onComputeCorrelation: (points: any[]) => void;
  onFinalizeCalibration: () => void;
  onRecordingNext: () => void;
  onTrimmingBack: () => void;
  onTrimmingSkip?: () => void;
  onProcessingBack: () => void;
  onProcessingSkip?: () => void;
  onCorrelatingBack: () => void;
  onFinalizingBack: () => void;
  onFinalizingSkip?: () => void;
}

export function CalibrationWorkflow({
  calibrationStep,
  uploadedImage,
  selectedFrames,
  uploadError,
  isUploading,
  onTypeSelect,
  onInstructionsNext,
  onTestRunSuccess,
  onTestRunBack,
  onFrameSelectionBack,
  onFramesSelected,
  onImageUpload,
  onCalibrationComplete,
  onCalibrationCancel,
  onUploadErrorDismiss,
  projectId,
  projectType,
  publisherMode,
  projectVideoFilename,
  hasVideoStream,
  videoCanvasRef,
  dronePosition,
  autoCalibrationRegion,
  autoCalibrationFrames,
  autoCalibrationError,
  autoCalibrationProgress,
  autoCalibrationMessage,
  onAutoRegionConfirm,
  onAutoImageSelect,
  onAutoCalibrationBack,
  calibrationSessionId,
  onSessionIdChange,
  calibrationSession,
  recordingStatus,
  recordingDuration,
  onRecordingDurationChange,
  onRecordingStatusChange,
  trimSegments,
  processingProgress,
  correlationPoints,
  transform,
  onStartRecording,
  onStopRecording,
  onApplyTrim,
  onRunProcessing,
  onComputeCorrelation,
  onFinalizeCalibration,
  onRecordingNext,
  onTrimmingBack,
  onTrimmingSkip,
  onProcessingBack,
  onProcessingSkip,
  onCorrelatingBack,
  onFinalizingBack,
  onFinalizingSkip,
}: CalibrationWorkflowProps) {
  const wrapOverlay = (content: React.ReactNode) => (
    <div className="fixed inset-0 z-[1200] bg-slate-50 flex flex-col">
      <div className="h-full w-full bg-white shadow-xl relative overflow-hidden flex flex-col">
        <button
          onClick={onCalibrationCancel}
          className="absolute top-4 right-4 text-muted-foreground hover:text-foreground z-10 p-2 hover:bg-slate-100 rounded-full transition-colors"
        >
          <span className="text-xl">✕</span>
        </button>
        <div className="flex-1 min-h-0">
          {content}
        </div>
      </div>
    </div>
  );

  // Type selection step
  if (calibrationStep === "type-selection") {
    return wrapOverlay(
      <CalibrationTypeSelection
        onSelect={onTypeSelect}
        onBack={onCalibrationCancel}
      />
    );
  }

  // Auto calibration steps
  if (calibrationStep === "auto-region") {
    return wrapOverlay(
      <AutoCalibrationRegion
        onConfirm={onAutoRegionConfirm}
        onBack={onAutoCalibrationBack}
        dronePosition={dronePosition}
        error={autoCalibrationError}
        progress={autoCalibrationProgress}
        message={autoCalibrationMessage}
      />
    );
  }

  if (calibrationStep === "auto-image-select") {
    return wrapOverlay(
      <AutoCalibrationImageSelect
        frames={autoCalibrationFrames}
        onSelect={onAutoImageSelect}
        onBack={onAutoCalibrationBack}
        error={autoCalibrationError}
        progress={autoCalibrationProgress}
        message={autoCalibrationMessage}
      />
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
            onClick={onInstructionsNext}
            className="w-full max-w-md bg-primary text-white py-3 rounded-lg font-bold flex items-center justify-center gap-2 shadow-lg shadow-primary/20"
          >
            <span className="w-4 h-4">▶</span>
            Продолжить
          </button>
        </div>
      </div>
    );
  }

  if (calibrationStep === "test-run" && projectId) {
    return wrapOverlay(
      <TestRunStep
        projectId={projectId}
        onSuccess={onTestRunSuccess}
        onBack={onTestRunBack}
        hasVideoStream={hasVideoStream}
        videoCanvasRef={videoCanvasRef}
      />
    );
  }

  if (calibrationStep === "frame-selection" && projectId) {
    return wrapOverlay(
      <FrameSelectionStep
        projectId={projectId}
        onFramesSelected={onFramesSelected}
        onBack={onFrameSelectionBack}
      />
    );
  }

  if (calibrationStep === "upload") {
    return wrapOverlay(
      <ImageUploadStep
        onUpload={onImageUpload}
        uploadError={uploadError}
        isUploading={isUploading}
        onErrorDismiss={onUploadErrorDismiss}
        onFileSelect={() => {}}
      />
    );
  }

  if (calibrationStep === "pairing") {
    if (selectedFrames.length > 0) {
      return (
        <CalibrationPointSelector
          images={selectedFrames}
          onComplete={onCalibrationComplete}
          onCancel={onCalibrationCancel}
          projectId={projectId}
        />
      );
    }
    if (uploadedImage) {
      return (
        <CalibrationPointSelector
          imageUrl={uploadedImage.url}
          onComplete={onCalibrationComplete}
          onCancel={onCalibrationCancel}
          projectId={projectId}
          imageFilename={uploadedImage.filename}
        />
      );
    }
  }

  if (calibrationStep === "recording") {
    const isSimulation =
      projectType === "симуляция" || publisherMode === "folder";
    if (isSimulation) {
      return wrapOverlay(
      <VideoUploadStep
        onComplete={onRecordingNext}
        onBack={onCalibrationCancel}
        onSkip={onRecordingNext}
        sessionId={calibrationSessionId}
        onSessionIdChange={onSessionIdChange}
        onRecordingStatusChange={onRecordingStatusChange}
        onRecordingDurationChange={onRecordingDurationChange}
        projectId={projectId}
        projectVideoFilename={projectVideoFilename}
      />
      );
    }
    return wrapOverlay(
      <VideoRecordingStep
        onComplete={onRecordingNext}
        onBack={onCalibrationCancel}
        onSkip={onRecordingNext}
        recordingStatus={recordingStatus}
        recordingDuration={recordingDuration}
        onRecordingDurationChange={onRecordingDurationChange}
        onRecordingStatusChange={onRecordingStatusChange}
        sessionId={calibrationSessionId}
        onSessionIdChange={onSessionIdChange}
      />
    );
  }

  if (calibrationStep === "trimming") {
    return wrapOverlay(
      <VideoTrimmingStep
        sessionId={calibrationSessionId || ""}
        onApplyTrim={onApplyTrim}
        onBack={onTrimmingBack}
        onSkip={onTrimmingSkip}
      />
    );
  }

  if (calibrationStep === "processing") {
    return wrapOverlay(
      <ProcessingStep
        sessionId={calibrationSessionId || ""}
        progress={processingProgress}
        onComplete={onRunProcessing}
        onBack={onProcessingBack}
        onSkip={onProcessingSkip}
      />
    );
  }

  if (calibrationStep === "correlating") {
    return wrapOverlay(
      <CorrelationStep
        sessionId={calibrationSessionId || ""}
        frames={calibrationSession?.frame_pose_data || []}
        onCompute={onComputeCorrelation}
        onBack={onCorrelatingBack}
      />
    );
  }

  if (calibrationStep === "finalizing") {
    return wrapOverlay(
      <FinalizeStep
        sessionId={calibrationSessionId || ""}
        transform={transform}
        onComplete={onFinalizeCalibration}
        onBack={onFinalizingBack}
        onSkip={onFinalizingSkip}
      />
    );
  }

  return null;
}
