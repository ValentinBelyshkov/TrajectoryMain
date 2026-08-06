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
  const [searchParams, setSearchParams] = useSearchParams();
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const hasRestoredRef = useRef(false);

  const controller = useProject(projectId);
  const {
    project,
    isLoading,
    error,
    isRecording,
    dronePosition,
    dronePath,
    telemetry,
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
    if (!project || hasRestoredRef.current) return;
    hasRestoredRef.current = true;

    const calibParam = searchParams.get("calib") as CalibrationStep | null;
    const sessionParam = searchParams.get("session");

    if (calibParam && RESTORABLE_STEPS.includes(calibParam)) {
      if (sessionParam) {
        setCalibrationSessionId(sessionParam);
        controller
          .resumeCalibrationSession(sessionParam)
          .then(() => {
            setCalibrationStep(calibParam);
          })
          .catch((e) => {
            console.error("Failed to resume calibration session:", e);
            setCalibrationStep(calibParam);
          });
      } else {
        setCalibrationStep(calibParam);
      }
    }
  }, [project, searchParams, setCalibrationStep, setCalibrationSessionId, controller]);

  // ── Sync calibration step → URL ───────────────────────────────────────
  useEffect(() => {
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
          onStartRecording={controller.startRecording}
          onStopRecording={controller.stopRecording}
          dronePosition={dronePosition}
          dronePath={dronePath}
          showCalibration={showCalibration}
          onCalibrate={controller.handleCalibrate}
          hasVideoStream={hasVideoStream}
          videoCanvasRef={videoCanvasRef}
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
