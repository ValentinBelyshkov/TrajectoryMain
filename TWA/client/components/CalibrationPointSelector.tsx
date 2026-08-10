import { useState, useRef } from "react";
import { Trash2, Save, MapPin } from "lucide-react";
import { Button } from "@/components/ui/button";
import { MapComponent } from "@/components/MapComponent";
import { saveAllGCPPoints } from "@/lib/api";
import { cn } from "@/lib/utils";

export interface CalibrationPoint {
  id: string;
  imageX: number;
  imageY: number;
  lat: number;
  lng: number;
  altitude: number;
}

interface CalibrationPointSelectorProps {
  imageUrl?: string;
  images?: { filename: string; url: string }[];
  onComplete: () => void;
  onCancel: () => void;
  projectId?: string;
  imageFilename?: string;
}

export function CalibrationPointSelector({
  imageUrl,
  images = [],
  onComplete,
  onCancel,
  projectId,
  imageFilename,
}: CalibrationPointSelectorProps) {
  // Effective list of snapshots the user can pick from.
  const effectiveImages =
    images.length > 0
      ? images
      : imageUrl && imageFilename
        ? [{ filename: imageFilename, url: imageUrl }]
        : [];

  // How many GCPs we need. The real calibration flow uses 5 distinct snapshots;
  // the legacy single-image path only has one, so it falls back to 1. Cap at the
  // number of available snapshots so it never becomes impossible to finish.
  const REQUIRED_GCPS =
    effectiveImages.length > 1 ? Math.min(5, effectiveImages.length) : 1;

  const [currentImageIndex, setCurrentImageIndex] = useState(0);
  const [pointsByImage, setPointsByImage] = useState<Record<string, CalibrationPoint[]>>({});

  const currentImage = effectiveImages[currentImageIndex];
  const completedPoints = currentImage ? pointsByImage[currentImage.filename] || [] : [];
  const imageHasPoint = completedPoints.length >= 1;

  const setCompletedPoints = (points: CalibrationPoint[]) => {
    if (!currentImage) return;
    setPointsByImage((prev) => ({
      ...prev,
      [currentImage.filename]: points,
    }));
  };

  const [pendingPoint, setPendingPoint] = useState<{ id: string } | null>(null);
  // The point the user clicked on the MAP (lat/lng). Kept separate from
  // pendingPoint so the confirm button can depend on it directly.
  const [mapPoint, setMapPoint] = useState<{ lat: number; lng: number } | null>(null);
  const [isSaving, setIsSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);

  const imageContainerRef = useRef<HTMLDivElement>(null);

  const totalCompleted = Object.values(pointsByImage).reduce(
    (acc, pts) => acc + pts.length,
    0,
  );

  // --- Flow: 1) select snapshot, 2) click a point on the MAP, 3) confirm ---
  // The point on the PHOTO is ALWAYS the image center — we never ask the user
  // to click the photo. We compute the center at confirm time.
  // Clicking the MAP directly arms the pending point and stores its lat/lng,
  // so no separate "arm" button is required.
  const handleMapClick = (lat: number, lng: number) => {
    if (imageHasPoint) return; // this snapshot already has its point
    if (!pendingPoint) setPendingPoint({ id: `point-${Date.now()}` });
    setMapPoint({ lat, lng });
  };

  const completePoint = (altitude: number = 0) => {
    if (!pendingPoint || !mapPoint || !currentImage) {
      alert("Сначала выберите точку на карте");
      return;
    }
    // The photo point is the center of the displayed image.
    const rect = imageContainerRef.current?.getBoundingClientRect();
    const cx = rect ? Math.round(rect.width / 2) : 0;
    const cy = rect ? Math.round(rect.height / 2) : 0;

    const newPoint: CalibrationPoint = {
      id: pendingPoint.id,
      imageX: cx,
      imageY: cy,
      lat: mapPoint.lat,
      lng: mapPoint.lng,
      altitude,
    };

    setCompletedPoints([...completedPoints, newPoint]);
    setPendingPoint(null);
    setMapPoint(null);
  };

  const cancelPoint = () => {
    setPendingPoint(null);
    setMapPoint(null);
  };

  const deletePoint = () => {
    // Remove the (single) point for the currently selected snapshot.
    setCompletedPoints([]);
  };

  const handleSaveGCP = async () => {
    if (totalCompleted !== REQUIRED_GCPS) {
      alert(`Нужно установить ровно ${REQUIRED_GCPS} точек (по одной на снимок). Сейчас: ${totalCompleted}`);
      return;
    }

    if (projectId) {
      setIsSaving(true);
      setSaveError(null);
      try {
        const payload = Object.entries(pointsByImage).map(([filename, pts]) => ({
          image_filename: filename,
          points: pts.map((p) => ({
            imageX: p.imageX,
            imageY: p.imageY,
            lat: p.lat,
            lng: p.lng,
            altitude: p.altitude,
          })),
        }));
        await saveAllGCPPoints(projectId, payload);
        onComplete();
      } catch (err) {
        setSaveError(err instanceof Error ? err.message : "Ошибка сохранения");
        setIsSaving(false);
      }
    } else {
      onComplete();
    }
  };

  const getProgressText = () => {
    if (pendingPoint && !mapPoint) {
      return `Точка ${totalCompleted + 1}/${REQUIRED_GCPS}: выберите точку на карте`;
    }
    if (pendingPoint && mapPoint) {
      return `Точка ${totalCompleted + 1}/${REQUIRED_GCPS}: подтвердите (точка на фото — в центре)`;
    }
    return `Выберите снимок и поставьте точку на карте (${totalCompleted}/${REQUIRED_GCPS})`;
  };

  // All collected GCPs as a flat list (for the side list).
  const allPoints = effectiveImages
    .map((img) => ({ img, pts: pointsByImage[img.filename] || [] }))
    .filter((x) => x.pts.length > 0);

  return (
    <div className="fixed inset-0 z-[1200] bg-slate-50 flex flex-col">
      <div className="h-full bg-white w-full flex flex-col shadow-xl overflow-hidden">
        {/* Header */}
        <div className="bg-gradient-to-r from-primary to-secondary text-white p-5 flex justify-between items-center shrink-0">
          <div>
            <h2 className="text-xl font-bold">Калибровка системы (GCP)</h2>
            <p className="text-white/80 text-sm">{getProgressText()}</p>
          </div>
          <Button variant="ghost" onClick={onCancel} className="text-white hover:bg-white/10">
            Отмена
          </Button>
        </div>

        {/* Content */}
        <div className="flex-1 flex gap-6 p-6 min-h-0">
          {/* Column 0: snapshot strip */}
          {effectiveImages.length > 1 && (
            <div className="w-28 flex flex-col gap-3 overflow-y-auto pr-2 border-r border-slate-100">
              {effectiveImages.map((img, idx) => {
                const done = (pointsByImage[img.filename]?.length || 0) >= 1;
                return (
                  <button
                    key={img.filename}
                    onClick={() => {
                      setPendingPoint(null);
                      setCurrentImageIndex(idx);
                    }}
                    className={cn(
                      "relative aspect-square rounded-lg overflow-hidden border-2 transition-all shrink-0",
                      currentImageIndex === idx
                        ? "border-primary ring-2 ring-primary/20"
                        : "border-transparent hover:border-slate-300",
                      done && "border-green-500",
                    )}
                  >
                    <img src={img.url} className="w-full h-full object-cover" alt="" />
                    <div className="absolute bottom-0 left-0 right-0 bg-black/50 text-white text-[8px] py-0.5 text-center">
                      {done ? "✓" : `${idx + 1}`}
                    </div>
                  </button>
                );
              })}
            </div>
          )}

          <div className="flex-1 grid grid-cols-1 lg:grid-cols-12 gap-6 min-h-0">
            {/* Column 1: Image (point is always at center) */}
            <div className="lg:col-span-5 flex flex-col min-h-0">
              <div className="mb-3 flex justify-between items-center">
                <h3 className="text-sm font-semibold flex items-center gap-2 truncate">
                  📷 {currentImage?.filename || "Изображение"}
                </h3>
                {imageHasPoint && (
                  <span className="text-[10px] text-green-600 bg-green-50 border border-green-200 rounded px-2 py-0.5">
                    Готово
                  </span>
                )}
              </div>

              <div
                ref={imageContainerRef}
                className="relative rounded-xl overflow-hidden border-2 border-slate-200 bg-slate-100 shadow-inner aspect-video lg:flex-1 pointer-events-none select-none"
              >
                {currentImage && (
                  <img
                    src={currentImage.url}
                    alt="Calibration"
                    className="w-full h-full object-contain pointer-events-none select-none"
                  />
                )}

                {/* The photo point is ALWAYS at the center of the image. */}
                <div
                  className="absolute w-7 h-7 rounded-full border-2 border-white bg-primary shadow-lg flex items-center justify-center text-white text-xs font-bold -translate-x-1/2 -translate-y-1/2"
                  style={{ left: "50%", top: "50%" }}
                  title="Точка на фото (центр)"
                >
                  <MapPin className="w-3.5 h-3.5" />
                </div>

                {/* Pending map point badge while choosing on map */}
                {pendingPoint && (
                  <div className="absolute top-2 left-2 bg-amber-500 text-white text-[10px] px-2 py-1 rounded shadow">
                    Выберите точку на карте →
                  </div>
                )}
              </div>
              <p className="text-[10px] text-slate-400 mt-2 text-center">
                Точка на фото фиксируется в центре кадра автоматически.
              </p>
            </div>

            {/* Column 2: Map */}
            <div className="lg:col-span-4 flex flex-col min-h-0">
              <div className="mb-3 flex justify-between items-center">
                <h3 className="text-sm font-semibold flex items-center gap-2">
                  🗺️ Карта
                  {pendingPoint && (
                    <span className="flex h-2 w-2 rounded-full bg-blue-500 animate-pulse" />
                  )}
                </h3>
              </div>

              <div className="relative aspect-video lg:flex-1 rounded-xl overflow-hidden border-2 border-slate-200 bg-slate-100 shadow-inner">
                <MapComponent
                  dronePosition={{ lat: 55.7558, lng: 37.6173 }}
                  followDrone={false}
                  path={allPoints.map((x) => ({
                    lat: x.pts[0].lat,
                    lng: x.pts[0].lng,
                  }))}
                  onMapClick={handleMapClick}
                  selectedPoint={
                    mapPoint
                      ? { lat: mapPoint.lat, lng: mapPoint.lng }
                      : completedPoints[0]
                        ? { lat: completedPoints[0].lat, lng: completedPoints[0].lng }
                        : undefined
                  }
                />
              </div>
            </div>

            {/* Column 3: collected points list */}
            <div className="lg:col-span-3 flex flex-col min-h-0 bg-slate-50 rounded-xl border border-slate-200 p-4">
              <h3 className="text-sm font-semibold mb-3">
                Точки ({totalCompleted}/{REQUIRED_GCPS})
              </h3>

              <div className="flex-1 overflow-y-auto space-y-2 pr-1">
                {allPoints.length === 0 && (
                  <div className="flex flex-col items-center justify-center h-32 text-slate-400 text-center">
                    <p className="text-xs">Точки ещё не добавлены</p>
                  </div>
                )}
                {allPoints.map(({ img, pts }, i) => (
                  <div
                    key={img.filename}
                    className="p-3 rounded-lg bg-white border border-slate-200 shadow-sm"
                  >
                    <div className="flex justify-between items-center mb-1">
                      <span className="font-bold text-sm text-slate-700">Точка {i + 1}</span>
                      {pts.length > 0 && (
                        <button
                          onClick={() => {
                            setCurrentImageIndex(
                              effectiveImages.findIndex((e) => e.filename === img.filename),
                            );
                            deletePoint();
                          }}
                          className="text-slate-300 hover:text-red-500 transition-colors"
                        >
                          <Trash2 className="w-3.5 h-3.5" />
                        </button>
                      )}
                    </div>
                    <div className="text-[10px] font-mono text-slate-500 space-y-0.5">
                      <p>IMG: {img.filename}</p>
                      <p>GPS: {pts[0].lat.toFixed(5)}, {pts[0].lng.toFixed(5)}</p>
                    </div>
                  </div>
                ))}
              </div>

              {/* Pending point editor */}
              {pendingPoint && (
                <div className="mt-4 p-3 bg-amber-50 border border-amber-200 rounded-lg">
                  <p className="text-xs font-bold text-amber-800 mb-2 uppercase tracking-wider">
                    Новая точка {totalCompleted + 1}
                  </p>
                  <div className="space-y-1.5 mb-3 text-[10px] font-mono">
                    <div className="flex justify-between">
                      <span className="text-amber-600">Фото:</span>
                      <span className="font-bold">центр</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-amber-600">GPS:</span>
                      <span className="font-bold">
                        {mapPoint ? `${mapPoint.lat.toFixed(5)}, ${mapPoint.lng.toFixed(5)}` : "—"}
                      </span>
                    </div>
                  </div>
                  <input
                    type="number"
                    step="0.1"
                    placeholder="Высота (м)"
                    defaultValue="0"
                    id="altitude-input"
                    className="w-full mb-3 px-2 py-1.5 border border-amber-200 rounded bg-white text-xs focus:ring-1 focus:ring-amber-500 outline-none"
                  />
                  <div className="grid grid-cols-2 gap-2">
                    <Button variant="ghost" size="sm" onClick={cancelPoint} className="h-8 text-xs">
                      Отмена
                    </Button>
                    <Button
                      size="sm"
                      onClick={() => {
                        const el = document.getElementById("altitude-input") as HTMLInputElement;
                        completePoint(parseFloat(el?.value || "0"));
                      }}
                      disabled={!mapPoint}
                      className="h-8 text-xs bg-amber-600 hover:bg-amber-700 text-white"
                    >
                      Подтвердить
                    </Button>
                  </div>
                </div>
              )}
            </div>
          </div>
        </div>

        {/* Footer */}
        <div className="p-6 border-t bg-slate-50 flex flex-col items-center shrink-0">
          {saveError && (
            <p className="text-[10px] text-red-500 mb-3 bg-red-50 p-2 rounded border border-red-100 max-w-md w-full">
              {saveError}
            </p>
          )}
          <Button
            onClick={handleSaveGCP}
            disabled={totalCompleted < REQUIRED_GCPS || isSaving}
            className="w-full max-w-md gap-2 h-12 font-bold text-sm shadow-lg shadow-primary/20"
          >
            {isSaving ? (
              <div className="animate-spin rounded-full h-4 w-4 border-2 border-white/30 border-t-white" />
            ) : (
              <Save className="w-4 h-4" />
            )}
            {isSaving ? "Сохранение..." : `Завершить (${totalCompleted}/${REQUIRED_GCPS})`}
          </Button>
          <p className="text-[10px] text-center text-slate-400 mt-3">
            Выберите 5 снимков, для каждого поставьте точку на карте и подтвердите.
            Точка на фото всегда в центре кадра.
          </p>
        </div>
      </div>
    </div>
  );
}
