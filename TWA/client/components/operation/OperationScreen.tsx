import { useEffect, useMemo, useRef, useState, type RefObject } from "react";
import { Checkbox } from "@/components/ui/checkbox";
import { MapComponent } from "@/components/MapComponent";
import type { DronePosition, DronePath, GPSStatus } from "@/hooks/useProject";

interface OperationScreenProps {
  isRecording: boolean;
  isBusy?: boolean;
  onStartRecording: () => void;
  onStopRecording: () => void;
  dronePosition: DronePosition;
  dronePath: DronePath;
  showCalibration: boolean;
  onCalibrate: () => void;
  hasVideoStream: boolean;
  videoCanvasRef: RefObject<HTMLCanvasElement | null>;
  saveFrames: boolean;
  onSaveFramesChange: (value: boolean) => void;
  gpsStatus?: GPSStatus;
  projectType?: string;
  systemStatus?: {
    status: "working" | "warning" | "not_working" | "error";
    publisher_mode: string;
    components: Record<string, string>;
  } | null;
}

export function OperationScreen({
  isRecording,
  isBusy = false,
  onStartRecording,
  onStopRecording,
  dronePosition,
  dronePath,
  showCalibration,
  onCalibrate,
  hasVideoStream,
  videoCanvasRef,
  saveFrames,
  onSaveFramesChange,
  gpsStatus,
  projectType,
  systemStatus,
}: OperationScreenProps) {
  // GPS logic: use correct property names
  // dronePosition uses "lng", while gpsStatus uses "lon"
  const gps = useMemo(() => {
    const lat = gpsStatus?.lat ?? dronePosition?.lat;
    const lon = gpsStatus?.lon ?? dronePosition?.lng;
    const alt = gpsStatus?.alt ?? null;

    const hasSignal =
      typeof lat === "number" &&
      typeof lon === "number" &&
      Number.isFinite(lat) &&
      Number.isFinite(lon);

    return { lat, lon, alt, hasSignal };
  }, [gpsStatus, dronePosition]);

  // ─── SLAM camera pose X/Y/Z ───
  const [camPose, setCamPose] = useState<{
    x: number;
    y: number;
    z: number;
  } | null>(null);

  const [poseError, setPoseError] = useState<string | null>(null);

  // Не допускаем одновременные запросы к API
  const poseRequestInFlightRef = useRef(false);

  // Автоматический запрос позы с частотой 10 Гц
  useEffect(() => {
    let cancelled = false;
    const abortController = new AbortController();

    const fetchCameraPose = async () => {
      if (cancelled || poseRequestInFlightRef.current) {
        return;
      }

      poseRequestInFlightRef.current = true;

      try {
        const response = await fetch("/api/ros2/topic/camera_pose", {
          signal: abortController.signal,
        });

        if (!response.ok) {
          throw new Error(`Ошибка HTTP: ${response.status}`);
        }

        const data = await response.json();

        if (data.error) {
          throw new Error(data.error);
        }

        const raw: string =
          typeof data.output === "string" ? data.output : "";

        if (!raw) {
          throw new Error("Пустой ответ с позой камеры");
        }

        // Берём x/y/z из блока position, а не из orientation
        const posBlock = raw.match(
          /position:\s*\{?[\s\S]*?\}/
        )?.[0] ?? raw;

        const px = posBlock.match(/x:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)/);
        const py = posBlock.match(/y:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)/);
        const pz = posBlock.match(/z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)/);

        if (!px || !py || !pz) {
          throw new Error("Поза не найдена в ответе");
        }

        const pose = {
          x: parseFloat(px[1]),
          y: parseFloat(py[1]),
          z: parseFloat(pz[1]),
        };

        if (
          !Number.isFinite(pose.x) ||
          !Number.isFinite(pose.y) ||
          !Number.isFinite(pose.z)
        ) {
          throw new Error("Получены некорректные координаты позы");
        }

        if (!cancelled) {
          setCamPose(pose);
          setPoseError(null);
        }
      } catch (error) {
        // Ошибку AbortController при размонтировании компонента не показываем
        if (
          error instanceof DOMException &&
          error.name === "AbortError"
        ) {
          return;
        }

        if (!cancelled) {
          setPoseError(
            error instanceof Error ? error.message : String(error)
          );
        }
      } finally {
        poseRequestInFlightRef.current = false;
      }
    };

    // Первый запрос выполняем сразу
    void fetchCameraPose();

    // 10 Гц = один запрос каждые 100 миллисекунд
    const intervalId = window.setInterval(() => {
      void fetchCameraPose();
    }, 100);

    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
      abortController.abort();
    };
  }, []);

  return (
    <div className="flex-1 flex flex-col lg:flex-row gap-4 p-4 lg:p-6 bg-gradient-to-br from-slate-50 to-blue-50 overflow-auto">
      {/* Left side - Camera Feed */}
      <div className="flex-1 flex flex-col gap-4">
        <div className="flex-1 bg-black rounded-lg border-2 border-border flex items-center justify-center relative overflow-hidden min-h-[300px] lg:min-h-0">
          {/* Video Canvas */}
          <canvas
            ref={videoCanvasRef}
            className={`absolute inset-0 w-full h-full object-contain ${
              hasVideoStream ? "opacity-100" : "opacity-0"
            }`}
            style={{ imageRendering: "pixelated" }}
          />

          {/* Placeholder when no video */}
          {!hasVideoStream && (
            <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
              <div className="text-center">
                <span className="text-6xl mb-4 block opacity-50">📷</span>
                <p className="text-white/60 text-sm">
                  Видеопоток с камеры дрона
                </p>
                <p className="text-white/40 text-xs mt-2">
                  Подключите дрон для начала трансляции
                </p>
              </div>
            </div>
          )}

          {/* Recording indicator */}
          {isRecording && (
            <div className="absolute top-4 right-4 flex items-center gap-2 bg-red-500 text-white px-3 py-1 rounded-full animate-pulse z-10">
              <div className="w-2 h-2 bg-white rounded-full animate-pulse" />
              <span className="font-semibold text-sm">Запись</span>
            </div>
          )}

          {/* GPS Warning */}
          {!gps.hasSignal && (
            <div className="absolute top-4 left-4 flex items-center gap-2 bg-amber-500 text-white px-3 py-1 rounded-full animate-pulse z-10">
              <div className="w-2 h-2 bg-white rounded-full" />
              <span className="font-semibold text-sm">
                ⚠️ Нет сигнала GPS
              </span>
            </div>
          )}
        </div>

        {/* Controls */}
        <div className="flex gap-3 flex-wrap flex-col">
          {!isRecording ? (
            <button
              onClick={onStartRecording}
              disabled={showCalibration || isBusy}
              className="flex-1 btn-primary py-3 flex items-center justify-center gap-2 disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <span className="text-lg">▶️</span>
              {isBusy ? "Запуск..." : "Старт"}
            </button>
          ) : (
            <button
              onClick={onStopRecording}
              disabled={isBusy}
              className="flex-1 bg-red-600 hover:bg-red-700 text-white font-semibold py-3 rounded-lg flex items-center justify-center gap-2 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <span className="text-lg">⏹️</span>
              {isBusy ? "Остановка..." : "Стоп"}
            </button>
          )}

          {!isRecording && (
            <div className="flex items-center gap-3 rounded-lg border border-border bg-white/60 px-4 py-3">
              <Checkbox
                id="save-frames"
                checked={saveFrames}
                onCheckedChange={(checked) =>
                  onSaveFramesChange(checked === true)
                }
              />

              <label
                htmlFor="save-frames"
                className="text-sm font-medium text-foreground cursor-pointer select-none"
              >
                запись кадров
              </label>
            </div>
          )}

          {!isRecording && (
            <button
              onClick={onCalibrate}
              className="flex-1 border-2 border-primary text-primary hover:bg-primary/5 font-semibold py-3 rounded-lg flex items-center justify-center gap-2 transition-colors"
            >
              <span className="text-lg">⚙️</span>
              Калибровка
            </button>
          )}
        </div>
      </div>

      {/* Right side - Map and Info */}
      <div className="w-full lg:w-[45%] flex flex-col gap-4">
        {/* Map */}
        <div className="flex-1 bg-white rounded-lg border-2 border-border relative overflow-hidden min-h-[300px] lg:min-h-0">
          <MapComponent
            dronePosition={dronePosition}
            path={dronePath}
            showMarkers={false}
          />
        </div>

        {/* Status Panel */}
        <div className="bg-white rounded-lg border border-border p-4">
          <h3 className="font-bold text-foreground mb-3">Информация</h3>

          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">
                Статус системы:
              </span>

              <span
                className={`font-semibold ${
                  systemStatus?.status === "working"
                    ? "text-green-600"
                    : systemStatus?.status === "warning"
                    ? "text-amber-600"
                    : systemStatus?.status === "error"
                    ? "text-red-600"
                    : "text-muted-foreground"
                }`}
              >
                {systemStatus?.status === "working"
                  ? "✓ Работает"
                  : systemStatus?.status === "warning"
                  ? "⚠️ Внимание"
                  : systemStatus?.status === "error"
                  ? "✗ Ошибка"
                  : systemStatus?.status === "not_working"
                  ? "✗ Не работает"
                  : "Загрузка..."}
              </span>
            </div>

            {systemStatus && (
              <div className="flex justify-between text-xs text-muted-foreground">
                <span>Режим:</span>

                <span className="font-medium">
                  {systemStatus.publisher_mode === "folder" ||
                  projectType === "симуляция"
                    ? "Симуляция (видео)"
                    : "Камера (RealSense)"}
                </span>
              </div>
            )}

            <div className="flex justify-between">
              <span className="text-muted-foreground">
                Калибровка:
              </span>

              <span
                className={`font-semibold ${
                  showCalibration
                    ? "text-amber-600"
                    : "text-green-600"
                }`}
              >
                {showCalibration ? "⏳ Требуется" : "✓ Выполнена"}
              </span>
            </div>

            <div className="flex justify-between">
              <span className="text-muted-foreground">Запись:</span>

              <span
                className={`font-semibold ${
                  isRecording
                    ? "text-red-600"
                    : "text-muted-foreground"
                }`}
              >
                {isRecording ? "● Запись" : "⊙ Ожидание"}
              </span>
            </div>

            {/* GPS Status */}
            <div className="flex justify-between">
              <span className="text-muted-foreground">
                GPS сигнал:
              </span>

              <span
                className={`font-semibold ${
                  gps.hasSignal
                    ? "text-green-600"
                    : "text-amber-600"
                }`}
              >
                {gps.hasSignal
                  ? `✓ ${gps.lat!.toFixed(5)}, ${gps.lon!.toFixed(5)}`
                  : "⚠️ Нет данных (>1с)"}
              </span>
            </div>

            {gps.hasSignal && gps.alt != null && (
              <div className="flex justify-between">
                <span className="text-muted-foreground">
                  Высота (GPS):
                </span>

                <span className="font-semibold text-blue-600">
                  {gps.alt.toFixed(2)} м
                </span>
              </div>
            )}

            {/* SLAM Pose X/Y/Z — автоматическое обновление 10 Гц */}
            <div className="border-t border-border pt-2 mt-2">
              <div className="flex justify-between items-center mb-2">
                <span className="text-muted-foreground">
                  Поза камеры (SLAM):
                </span>

                <span className="text-xs text-muted-foreground">
                  обновление 10 Гц
                </span>
              </div>

              {camPose ? (
                <div className="grid grid-cols-3 gap-2 text-center">
                  <div className="bg-slate-50 rounded border border-slate-200 py-1">
                    <div className="text-[10px] uppercase text-muted-foreground">
                      X
                    </div>

                    <div className="font-semibold text-blue-600">
                      {camPose.x.toFixed(3)}
                    </div>
                  </div>

                  <div className="bg-slate-50 rounded border border-slate-200 py-1">
                    <div className="text-[10px] uppercase text-muted-foreground">
                      Y
                    </div>

                    <div className="font-semibold text-blue-600">
                      {camPose.y.toFixed(3)}
                    </div>
                  </div>

                  <div className="bg-slate-50 rounded border border-slate-200 py-1">
                    <div className="text-[10px] uppercase text-muted-foreground">
                      Z
                    </div>

                    <div className="font-semibold text-blue-600">
                      {camPose.z.toFixed(3)}
                    </div>
                  </div>
                </div>
              ) : (
                <div className="text-xs text-muted-foreground">
                  Ожидание данных позы...
                </div>
              )}

              {poseError && (
                <div className="text-xs text-red-600 mt-1">
                  ⚠️ {poseError}
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}