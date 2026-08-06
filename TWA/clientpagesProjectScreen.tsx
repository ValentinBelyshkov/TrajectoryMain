import { useNavigate, useParams } from "react-router-dom";
import { useState } from "react";
import { useQuery } from "@tanstack/react-query";
import { TelemetryBar } from "@/components/TelemetryBar";
import { Button } from "@/components/ui/button";
import { getProject } from "@/lib/api";
import { useProject } from "@/hooks/useProject";
import { ProjectHeader } from "@/components/ProjectHeader";
import { CalibrationWorkflow } from "@/components/calibration/CalibrationWorkflow";
import { OperationScreen } from "@/components/operation/OperationScreen";
import { SettingsModal } from "@/components/SettingsModal";

export default function ProjectScreen() {
  const { projectId } = useParams();
  const navigate = useNavigate();
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);

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
    setCalibrationStep,
  } = useProject(projectId);

  const onRecordingNext = () => setCalibrationStep("trimming");
  const onTrimmingBack = () => setCalibrationStep("recording");
  const onProcessingBack = () => setCalibrationStep("trimming");
  const onCorrelatingBack = () => setCalibrationStep("processing");
  const onFinalizingBack = () => setCalibrationStep("correlating");

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
          onRecordingNext={() => setCalibrationStep("trimming")}
          onTrimmingBack={() => setCalibrationStep("recording")}
          onProcessingBack={() => setCalibrationStep("trimming")}
          onCorrelatingBack={() => setCalibrationStep("processing")}
          onFinalizingBack={() => setCalibrationStep("correlating")}
        />
      )}

      <SettingsModal
        open={isSettingsOpen}
        onOpenChange={setIsSettingsOpen}
      />
    </div>
  );
}
