import { useNavigate, useParams, useSearchParams } from "react-router-dom";
import { useState, useEffect, useRef } from "react";
import { Button } from "@/components/ui/button";
import { useProject } from "@/hooks/useProject";
import type { CalibrationStep } from "@/hooks/useProject";
import { ProjectHeader } from "@/components/ProjectHeader";
import { CalibrationWorkflow } from "@/components/calibration/CalibrationWorkflow";
import { OperationScreen } from "@/components/operation/OperationScreen";
import { SettingsModal } from "@/components/SettingsModal";
import { CalibrationProvider } from "@/contexts/CalibrationContext";
import { controlTerraSLAMComponent } from "@/lib/api";

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
  const [, setSearchParams] = useSearchParams();
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const hasRestoredRef = useRef(false);

  // Snapshot of the calibration params as they were when the page was opened.
  // This must be read once, synchronously on the first render: the
  // "step → URL" effect below runs before the project finishes loading and
  // would otherwise strip ?calib/?session from the URL (calibrationStep is
  // still "idle" at that point), losing the state we want to restore.
  const [initialCalib] = useState(() => {
    const params = new URLSearchParams(window.location.search);
    const step = params.get("calib") as CalibrationStep | null;
    return {
      step: step && RESTORABLE_STEPS.includes(step) ? step : null,
      session: params.get("session"),
    };
  });

  // While a restore is pending, the URL is the source of truth and must not be
  // overwritten by the (still default) calibration step.
  const [isRestorePending, setIsRestorePending] = useState(
    () => initialCalib.step !== null,
  );

  const controller = useProject(projectId);
  const {
    project,
    isLoading,
    error,
    isRecording,
    dronePosition,
    dronePath,
    showCalibration,
    hasVideoStream,
    videoCanvasRef,
    gpsStatus,
    systemStatus,
    calibrationStep,
    setCalibrationStep,
    calibrationSessionId,
    setCalibrationSessionId,
  } = controller;

  // ── Restore calibration step from URL on project load (once) ─────────
  useEffect(() => {
    if (!isRestorePending || hasRestoredRef.current) return;
    if (!project) return;
    hasRestoredRef.current = true;

    const calibParam = initialCalib.step;
    const sessionParam = initialCalib.session;
    if (!calibParam) {
      setIsRestorePending(false);
      return;
    }

    // Switch to the calibration step immediately so the user never sees the
    // main screen flash while the session payload is being fetched. The
    // session data is loaded in the background; steps that need it re-read it
    // once it arrives.
    if (sessionParam) {
      setCalibrationSessionId(sessionParam);
      controller.resumeCalibrationSession(sessionParam).catch((e) => {
        console.error("Failed to resume calibration session:", e);
      });
    }
    setCalibrationStep(calibParam);
    setIsRestorePending(false);
  }, [
    project,
    isRestorePending,
    initialCalib,
    setCalibrationStep,
    setCalibrationSessionId,
    controller,
  ]);

  // ── Sync calibration step → URL ───────────────────────────────────────
  useEffect(() => {
    // Do not touch the URL until the pending restore has been applied,
    // otherwise a page refresh wipes ?calib/?session before they are read.
    if (isRestorePending) return;

    if (!RESTORABLE_STEPS.includes(calibrationStep)) {
      setSearchParams(
        (prev) => {
          const next = new URLSearchParams(prev);
          next.delete("calib");
          next.delete("session");
          return next;
        },
        { replace: true },
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
        { replace: true },
      );
    }
  }, [calibrationStep, calibrationSessionId, isRestorePending]); // eslint-disable-line react-hooks/exhaustive-deps

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

      {calibrationStep === "idle" || calibrationStep === "complete" ? (
        <OperationScreen
          isRecording={isRecording}
          isBusy={controller.isBusy}
          onStartRecording={controller.startRecording}
          onStopRecording={controller.stopRecording}
          dronePosition={dronePosition}
          dronePath={dronePath}
          showCalibration={showCalibration}
          onCalibrate={controller.handleCalibrate}
          hasVideoStream={hasVideoStream}
          videoCanvasRef={videoCanvasRef}
          saveFrames={controller.saveFrames}
          onSaveFramesChange={controller.setSaveFrames}
          gpsStatus={gpsStatus}
          projectType={project?.type}
          systemStatus={systemStatus}
        />
      ) : (
        <CalibrationProvider value={controller}>
          <CalibrationWorkflow
            onStartPublisher={async (pid: string) => {
              await controlTerraSLAMComponent("publisher:realsense", "start", pid);
            }}
          />
        </CalibrationProvider>
      )}

      <SettingsModal open={isSettingsOpen} onOpenChange={setIsSettingsOpen} />
    </div>
  );
}
