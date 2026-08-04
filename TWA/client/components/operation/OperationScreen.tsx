import { useRef, useMemo, type RefObject } from "react";
import { MapComponent } from "@/components/MapComponent";
import type { DronePosition, DronePath, GPSStatus } from "@/hooks/useProject";

interface OperationScreenProps {
  isRecording: boolean;
  onStartRecording: () => void;
  onStopRecording: () => void;
  dronePosition: DronePosition;
  dronePath: DronePath;
  showCalibration: boolean;
  onCalibrate: () => void;
  hasVideoStream: boolean;
  videoCanvasRef: RefObject<HTMLCanvasElement | null>; 
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
  onStartRecording,
  onStopRecording,
  dronePosition,
  dronePath,
  showCalibration,
  onCalibrate,
  hasVideoStream,
  videoCanvasRef,
  gpsStatus,
  projectType,
  systemStatus,
}: OperationScreenProps) {

  // GPS logic: use correct property names (dronePosition uses "lng" not "lon")
  const gps = useMemo(() => {
    const lat = gpsStatus?.lat ?? dronePosition?.lat;
    const lon = gpsStatus?.lon ?? dronePosition?.lng;
    const alt = gpsStatus?.alt ?? null;
    const hasSignal = typeof lat === "number" && typeof lon === "number";
    return { lat, lon, alt, hasSignal };
  }, [gpsStatus, dronePosition]);

  return (
    <div className="flex-1 flex flex-col lg:flex-row gap-4 p-4 lg:p-6 bg-gradient-to-br from-slate-50 to-blue-50 overflow-auto">
      {/* Left side - Camera Feed */}
      <div className="flex-1 flex flex-col gap-4">
        <div className="flex-1 bg-black rounded-lg border-2 border-border flex items-center justify-center relative overflow-hidden min-h-[300px] lg:min-h-0">
          
          {/* Video Canvas - fixed styling for proper display */}
          <canvas
            ref={videoCanvasRef}
            className={`absolute inset-0 w-full h-full object-contain ${hasVideoStream ? 'opacity-100' : 'opacity-0'}`}
            style={{ imageRendering: 'pixelated' }}
          />
          
          {/* Placeholder when no video */}
          {!hasVideoStream && (
            <div className="absolute inset-0 flex items-center justify-center bg-gray-900">
              <div className="text-center">
                <span className="text-6xl mb-4 block opacity-50">📷</span>
                <p className="text-white/60 text-sm">Видеопоток с камеры дрона</p>
                <p className="text-white/40 text-xs mt-2">Подключите дрон для начала трансляции</p>
              </div>
            </div>
          )}

          {/* Recording indicator only */}
          {isRecording && (
            <div className="absolute top-4 right-4 flex items-center gap-2 bg-red-500 text-white px-3 py-1 rounded-full animate-pulse z-10">
              <div className="w-2 h-2 bg-white rounded-full animate-pulse"></div>
              <span className="font-semibold text-sm">Запись</span>
            </div>
          )}

          {/* GPS Warning - only if NO valid coordinates */}
          {!gps.hasSignal && (
            <div className="absolute top-4 left-4 flex items-center gap-2 bg-amber-500 text-white px-3 py-1 rounded-full animate-pulse z-10">
              <div className="w-2 h-2 bg-white rounded-full"></div>
              <span className="font-semibold text-sm">⚠️ Нет сигнала GPS</span>
            </div>
          )}
          
          {/* ✅ REMOVED: Green GPS coordinate overlay from video feed */}
        </div>

        {/* Controls */}
        <div className="flex gap-3 flex-wrap flex-col">
          {!isRecording ? (
            <button
              onClick={onStartRecording}
              disabled={!gps.hasSignal || showCalibration}
              className="flex-1 btn-primary py-3 flex items-center justify-center gap-2 disabled:opacity-50 disabled:cursor-not-allowed"
            >
              <span className="text-lg">▶️</span>
              Поднять и начать запись
            </button>
          ) : (
            <button
              onClick={onStopRecording}
              className="flex-1 bg-red-600 hover:bg-red-700 text-white font-semibold py-3 rounded-lg flex items-center justify-center gap-2 transition-colors"
            >
              <span className="text-lg">⏹️</span>
              Остановить запись
            </button>
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
          <MapComponent dronePosition={dronePosition} path={dronePath} showMarkers={false} />
        </div>

        {/* Status Panel - GPS info stays here */}
        <div className="bg-white rounded-lg border border-border p-4">
          <h3 className="font-bold text-foreground mb-3">Информация</h3>
          <div className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Статус системы:</span>
              <span className={`font-semibold ${
                systemStatus?.status === "working" ? "text-green-600" :
                systemStatus?.status === "warning" ? "text-amber-600" :
                systemStatus?.status === "error" ? "text-red-600" :
                "text-muted-foreground"
              }`}>
                {systemStatus?.status === "working" ? "✓ Работает" :
                 systemStatus?.status === "warning" ? "⚠️ Внимание" :
                 systemStatus?.status === "error" ? "✗ Ошибка" :
                 systemStatus?.status === "not_working" ? "✗ Не работает" :
                 "Загрузка..."}
              </span>
            </div>
            {systemStatus && (
              <div className="flex justify-between text-xs text-muted-foreground">
                <span>Режим:</span>
                <span className="font-medium">
                  {systemStatus.publisher_mode === "folder" || projectType === "симуляция" 
                    ? "Симуляция (видео)" 
                    : "Камера (RealSense)"}
                </span>
              </div>
            )}
            <div className="flex justify-between">
              <span className="text-muted-foreground">Калибровка:</span>
              <span className={`font-semibold ${showCalibration ? "text-amber-600" : "text-green-600"}`}>
                {showCalibration ? "⏳ Требуется" : "✓ Выполнена"}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Запись:</span>
              <span className={`font-semibold ${isRecording ? "text-red-600" : "text-muted-foreground"}`}>
                {isRecording ? "● Запись" : "⊙ Ожидание"}
              </span>
            </div>
            
            {/* GPS Status - displayed ONLY in panel, not on video */}
            <div className="flex justify-between">
              <span className="text-muted-foreground">GPS сигнал:</span>
              <span className={`font-semibold ${gps.hasSignal ? "text-green-600" : "text-amber-600"}`}>
                {gps.hasSignal 
                  ? `✓ ${gps.lat.toFixed(5)}, ${gps.lon.toFixed(5)}` 
                  : "⚠️ Нет данных (>1с)"}
              </span>
            </div>
            
            {gps.hasSignal && gps.alt != null && (
              <div className="flex justify-between">
                <span className="text-muted-foreground">Высота (GPS):</span>
                <span className="font-semibold text-blue-600">
                  {gps.alt.toFixed(2)} м
                </span>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
