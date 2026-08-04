import { useNavigate, useParams, useSearchParams } from "react-router-dom";
import { useState, useEffect, useRef } from "react";
import { TelemetryBar } from "@/components/TelemetryBar";
import { Button } from "@/components/ui/button";
import { useProject } from "@/hooks/useProject";
import type { CalibrationStep } from "@/hooks/useProject";
import { ProjectHeader } from "@/components/ProjectHeader";
import { CalibrationWorkflow } from "@/components/calibration/CalibrationWorkflow";
import { OperationScreen } from "@/components/operation/OperationScreen";
import { SettingsModal } from "@/components/SettingsModal";

const RESTORABLE_STEPS: CalibrationStep[] = [
  "type-selection",
  "recording",
  "trimming",
  "processing",
  "correlating",
  "finalizing",
];

export default function ProjectScreen() {
  const { projectId } = useParams();
  const navigate = useNavigate();
  const [searchParams, setSearchParams] = useSearchParams();
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const hasRestoredRef = useRef(false);

  const {
    project,
    isLoading,
    error,
    isRecording,
    dronePosition,
    dronePath,
    telemetry,
    showCalibration,
    calibrationStep,
    setCalibrationStep,
    uploadedImage,
    selectedFrames,
    uploadError,
    isUploading,
    hasVideoStream,
    videoCanvasRef,
    startRecording,
    stopRecording,
    handleCalibrate,
    handleCalibrationTypeSelect,
    handleInstructionsNext,
    handleTestRunSuccess,
    handleTestRunBack,
    handleFrameSelectionBack,
    handleFramesSelected,
    handleImageUpload,
    handleCalibrationComplete,
    handleCalibrationCancel,
    clearUploadError,
    gpsStatus,
    systemStatus,
    // Auto calibration
    autoCalibrationRegion,
    autoCalibrationFrames,
    autoCalibrationError,
    autoCalibrationProgress,
    autoCalibrationMessage,
    handleAutoRegionConfirm,
    handleAutoImageSelect,
    handleAutoCalibrationBack,
    // NEW calibration workflow
    calibrationSessionId,
    setCalibrationSessionId,
    calibrationSession,
    recordingStatus,
    recordingDuration,
    setRecordingDuration,
    setRecordingStatus,
    trimSegments,
    processingProgress,
    correlationPoints,
    transform,
    handleStartRecording,
    handleStopRecording,
    handleApplyTrim,
    handleRunProcessing,
    handleComputeCorrelation,
    handleFinalizeCalibration,
  } = useProject(projectId);

  // ── Step transitions ──────────────────────────────────────────────────
  const onRecordingNext   = () => setCalibrationStep("trimming");
  const onTrimmingBack    = () => setCalibrationStep("recording");
  const onTrimmingSkip    = () => setCalibrationStep("processing");
  const onProcessingBack  = () => setCalibrationStep("trimming");
  const onProcessingSkip  = () => setCalibrationStep("correlating");
  const onCorrelatingBack = () => setCalibrationStep("processing");
  const onFinalizingBack  = () => setCalibrationStep("correlating");
  const onFinalizingSkip  = () => setCalibrationStep("complete");

  // ── Restore calibration step from URL on project load (once) ─────────
  useEffect(() => {
    if (!project || hasRestoredRef.current) return;
    hasRestoredRef.current = true;

    const calibParam = searchParams.get("calib") as CalibrationStep | null;
    const sessionParam = searchParams.get("session");

    if (calibParam && RESTORABLE_STEPS.includes(calibParam)) {
      setCalibrationStep(calibParam);
      if (sessionParam) setCalibrationSessionId(sessionParam);
    }
  }, [project]); // eslint-disable-line react-hooks/exhaustive-deps

  // ── Sync calibration step → URL ───────────────────────────────────────
  useEffect(() => {
    if (!RESTORABLE_STEPS.includes(calibrationStep)) {
      // Clear calib params when idle / complete
      setSearchParams(
        (prev) => {
          const next = new URLSearchParams(prev);
          next.delete("calib");
          next.delete("session");
          return next;
        },
        { replace: true }
      );
    } else {
      setSearchParams(
        (prev) => {
          const next = new URLSearchParams(prev);
          next.set("calib", calibrationStep);
          if (calibrationSessionId) {
            next.set("session", calibrationSessionId);
          } else {
            next.delete("session");
          }
          return next;
        },
        { replace: true }
      );
    }
  }, [calibrationStep, calibrationSessionId]); // eslint-disable-line react-hooks/exhaustive-deps

  // ── Loading / error states ────────────────────────────────────────────
  if (isLoading) {
    return (
      <div className="min-h-screen bg-background flex items-center justify-center">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mx-auto mb-4"></div>
          <p className="text-muted-foreground">Загрузка проекта...</p>
        </div>
      </div>
    );
  }

  if (error || !project) {
    return (
      <div className="min-h-screen bg-background flex items-center justify-center">
        <div className="text-center">
          <div className="text-6xl mb-4">❌</div>
          <h2 className="text-2xl font-bold text-foreground mb-2">
            Проект не найден
          </h2>
          <p className="text-muted-foreground mb-6">
            Не удалось загрузить данные проекта
          </p>
          <Button onClick={() => navigate("/")} className="btn-primary">
            Вернуться к списку проектов
          </Button>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-background flex flex-col">
      <ProjectHeader
        project={project}
        onBack={() => navigate("/")}
        onSettingsClick={() => setIsSettingsOpen(true)}
      />

      <TelemetryBar data={telemetry} />

      {calibrationStep === "idle" || calibrationStep === "complete" ? (
        <OperationScreen
          isRecording={isRecording}
          onStartRecording={startRecording}
          onStopRecording={stopRecording}
          dronePosition={dronePosition}
          dronePath={dronePath}
          showCalibration={showCalibration}
          onCalibrate={handleCalibrate}
          hasVideoStream={hasVideoStream}
          videoCanvasRef={videoCanvasRef}
          gpsStatus={gpsStatus}
          projectType={project?.type}
          systemStatus={systemStatus}
        />
      ) : (
        <CalibrationWorkflow
          calibrationStep={calibrationStep}
          uploadedImage={uploadedImage}
          selectedFrames={selectedFrames}
          uploadError={uploadError}
          isUploading={isUploading}
          onTypeSelect={handleCalibrationTypeSelect}
          onInstructionsNext={handleInstructionsNext}
          onTestRunSuccess={handleTestRunSuccess}
          onTestRunBack={handleTestRunBack}
          onFrameSelectionBack={handleFrameSelectionBack}
          onFramesSelected={handleFramesSelected}
          onImageUpload={handleImageUpload}
          onCalibrationComplete={handleCalibrationComplete}
          onCalibrationCancel={handleCalibrationCancel}
          onUploadErrorDismiss={clearUploadError}
          projectId={projectId}
          projectType={project?.type}
          publisherMode={systemStatus?.publisher_mode}
          projectVideoFilename={project?.videoFilename}
          hasVideoStream={hasVideoStream}
          videoCanvasRef={videoCanvasRef}
          dronePosition={dronePosition}
          autoCalibrationRegion={autoCalibrationRegion}
          autoCalibrationFrames={autoCalibrationFrames}
          autoCalibrationError={autoCalibrationError}
          autoCalibrationProgress={autoCalibrationProgress}
          autoCalibrationMessage={autoCalibrationMessage}
          onAutoRegionConfirm={handleAutoRegionConfirm}
          onAutoImageSelect={handleAutoImageSelect}
          onAutoCalibrationBack={handleAutoCalibrationBack}
          calibrationSessionId={calibrationSessionId}
          onSessionIdChange={setCalibrationSessionId}
          calibrationSession={calibrationSession}
          recordingStatus={recordingStatus}
          recordingDuration={recordingDuration}
          onRecordingDurationChange={setRecordingDuration}
          onRecordingStatusChange={setRecordingStatus}
          trimSegments={trimSegments}
          processingProgress={processingProgress}
          correlationPoints={correlationPoints}
          transform={transform}
          onStartRecording={handleStartRecording}
          onStopRecording={handleStopRecording}
          onApplyTrim={handleApplyTrim}
          onRunProcessing={handleRunProcessing}
          onComputeCorrelation={handleComputeCorrelation}
          onFinalizeCalibration={handleFinalizeCalibration}
          onRecordingNext={onRecordingNext}
          onTrimmingBack={onTrimmingBack}
          onTrimmingSkip={onTrimmingSkip}
          onProcessingBack={onProcessingBack}
          onProcessingSkip={onProcessingSkip}
          onCorrelatingBack={onCorrelatingBack}
          onFinalizingBack={onFinalizingBack}
          onFinalizingSkip={onFinalizingSkip}
        />
      )}

      <SettingsModal
        open={isSettingsOpen}
        onOpenChange={setIsSettingsOpen}
      />
    </div>
  );
}
