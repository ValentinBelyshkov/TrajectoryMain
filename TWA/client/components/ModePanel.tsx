import { useState } from "react";
import {
  Play,
  Square,
  StopCircle,
  Settings,
  AlertTriangle,
} from "lucide-react";
import { Button } from "./ui/button";
import { cn } from "@/lib/utils";

type DroneMode = "manual" | "auto" | "simulation" | "hover";
type CalibrationStatus = "not_calibrated" | "pending" | "calibrated";

interface ModePanelProps {
  isRecording?: boolean;
  mode?: DroneMode;
  calibrationStatus?: CalibrationStatus;
  onStartRecording?: () => void;
  onStopRecording?: () => void;
  onModeChange?: (mode: DroneMode) => void;
  onEmergencyStop?: () => void;
}

export function ModePanel({
  isRecording = false,
  mode = "manual",
  calibrationStatus = "not_calibrated",
  onStartRecording,
  onStopRecording,
  onModeChange,
  onEmergencyStop,
}: ModePanelProps) {
  const [showSettings, setShowSettings] = useState(false);

  const modeLabels: Record<DroneMode, string> = {
    manual: "Ручной",
    auto: "Автоматический",
    simulation: "Симуляция",
    hover: "Зависание",
  };

  const calibrationLabels: Record<CalibrationStatus, string> = {
    not_calibrated: "Не откалибровано",
    pending: "Калибровка...",
    calibrated: "Откалибровано",
  };

  const isCalibrated = calibrationStatus === "calibrated";

  return (
    <div className="bg-white border border-border rounded-lg p-4 space-y-4">
      {/* Calibration Status */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <div
            className={cn(
              "w-3 h-3 rounded-full",
              isCalibrated ? "bg-green-500" : "bg-amber-500",
            )}
          />
          <span className="text-sm font-medium text-foreground">
            {calibrationLabels[calibrationStatus]}
          </span>
        </div>
        {!isCalibrated && (
          <span className="text-xs text-amber-600">Требуется калибровка</span>
        )}
      </div>

      {/* Mode Selection */}
      <div className="space-y-2">
        <label className="text-xs font-semibold text-muted-foreground uppercase tracking-wider">
          Режим полета
        </label>
        <div className="grid grid-cols-2 gap-2">
          {(["manual", "auto", "hover", "simulation"] as DroneMode[]).map(
            (m) => (
              <button
                key={m}
                onClick={() => onModeChange?.(m)}
                className={cn(
                  "px-3 py-2 rounded-lg text-sm font-medium transition-colors",
                  mode === m
                    ? "bg-primary text-white"
                    : "bg-muted text-foreground hover:bg-muted/80",
                )}
              >
                {modeLabels[m]}
              </button>
            ),
          )}
        </div>
      </div>

      {/* Recording Controls */}
      <div className="flex gap-2">
        {!isRecording ? (
          <Button
            onClick={onStartRecording}
            disabled={!isCalibrated}
            className="flex-1 gap-2"
          >
            <Play className="w-4 h-4" />
            Старт
          </Button>
        ) : (
          <Button
            onClick={onStopRecording}
            variant="destructive"
            className="flex-1 gap-2"
          >
            <Square className="w-4 h-4" />
            Стоп
          </Button>
        )}

        <Button
          variant="outline"
          size="icon"
          onClick={() => setShowSettings(!showSettings)}
          title="Настройки"
        >
          <Settings className="w-4 h-4" />
        </Button>

        <Button
          variant="outline"
          size="icon"
          className="border-red-500 text-red-500 hover:bg-red-50"
          onClick={onEmergencyStop}
          title="Экстренная остановка"
        >
          <StopCircle className="w-4 h-4" />
        </Button>
      </div>

      {/* Warning if not calibrated */}
      {!isCalibrated && (
        <div className="flex items-start gap-2 p-3 bg-amber-50 border border-amber-200 rounded-lg">
          <AlertTriangle className="w-5 h-5 text-amber-500 flex-shrink-0 mt-0.5" />
          <div>
            <p className="text-sm font-medium text-amber-900">
              Калибровка не выполнена
            </p>
            <p className="text-xs text-amber-700">
              Пожалуйста, выполните калибровку перед началом полета
            </p>
          </div>
        </div>
      )}
    </div>
  );
}
