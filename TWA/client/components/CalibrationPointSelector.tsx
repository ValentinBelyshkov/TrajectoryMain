import { useState, useRef } from "react";
import { Trash2, Save, GripVertical } from "lucide-react";
import { Button } from "@/components/ui/button";
import { MapComponent } from "@/components/MapComponent";
import { saveAllGCPPoints, type CalibrationPointRequest } from "@/lib/api";
import { motion, Reorder, AnimatePresence } from "framer-motion";
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

interface PendingPoint {
  id: string;
  imageX?: number;
  imageY?: number;
  lat?: number;
  lng?: number;
}

export function CalibrationPointSelector({
  imageUrl,
  images = [],
  onComplete,
  onCancel,
  projectId,
  imageFilename,
}: CalibrationPointSelectorProps) {
  const [currentImageIndex, setCurrentImageIndex] = useState(0);
  const [pointsByImage, setPointsByImage] = useState<Record<string, CalibrationPoint[]>>({});
  
  // For legacy support or single image
  const effectiveImages = images.length > 0 ? images : (imageUrl && imageFilename ? [{ filename: imageFilename, url: imageUrl }] : []);
  const currentImage = effectiveImages[currentImageIndex];

  const completedPoints = currentImage ? (pointsByImage[currentImage.filename] || []) : [];
  
  const setCompletedPoints = (points: CalibrationPoint[] | ((prev: CalibrationPoint[]) => CalibrationPoint[])) => {
    if (!currentImage) return;
    setPointsByImage(prev => {
      const currentPoints = prev[currentImage.filename] || [];
      const nextPoints = typeof points === 'function' ? points(currentPoints) : points;
      return { ...prev, [currentImage.filename]: nextPoints };
    });
  };

  const [pendingPoint, setPendingPoint] = useState<PendingPoint | null>(null);
  const [currentMode, setCurrentMode] = useState<"image" | "map" | null>(null);
  const [isSaving, setIsSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  
  const imageContainerRef = useRef<HTMLDivElement>(null);

  const REQUIRED_POINTS = 1;
  const pointNumber = completedPoints.length + 1;

  const startNewPoint = (mode: "image" | "map") => {
    if (completedPoints.length >= REQUIRED_POINTS) return;
    if (pendingPoint) return;

    setCurrentMode(mode);
    setPendingPoint({
      id: `point-${Date.now()}`,
    });
  };

  const handleImageClick = (e: React.MouseEvent<HTMLDivElement>) => {
    if (completedPoints.length >= REQUIRED_POINTS) return;
    if (pendingPoint?.imageX && currentMode === "map") return;

    const rect = e.currentTarget.getBoundingClientRect();
    const x = Math.round(e.clientX - rect.left);
    const y = Math.round(e.clientY - rect.top);

    setCurrentMode("map");
    setPendingPoint({
      id: `point-${Date.now()}`,
      imageX: x,
      imageY: y,
    });
  };

  const handleMapPoint = (lat: number, lng: number) => {
    if (!pendingPoint || currentMode !== "map") return;

    setPendingPoint((prev) => {
      if (!prev) return null;
      return { ...prev, lat, lng };
    });
  };

  const handleMapClick = (lat: number, lng: number) => {
    handleMapPoint(lat, lng);
  };

  const completePoint = (altitude: number = 0) => {
    if (
      !pendingPoint ||
      !pendingPoint.imageX ||
      !pendingPoint.imageY ||
      !pendingPoint.lat ||
      !pendingPoint.lng
    ) {
      alert("Требуются координаты на обеих сторонах");
      return;
    }

    const newPoint: CalibrationPoint = {
      id: pendingPoint.id,
      imageX: pendingPoint.imageX,
      imageY: pendingPoint.imageY,
      lat: pendingPoint.lat,
      lng: pendingPoint.lng,
      altitude,
    };

    setCompletedPoints([...completedPoints, newPoint]);
    setPendingPoint(null);
    setCurrentMode(null);
  };

  const cancelPoint = () => {
    setPendingPoint(null);
    setCurrentMode(null);
  };

  const deletePoint = (index: number) => {
    setCompletedPoints(completedPoints.filter((_, i) => i !== index));
  };

  const updatePointPosition = (id: string, x: number, y: number) => {
    setCompletedPoints((prev) =>
      prev.map((p) => (p.id === id ? { ...p, imageX: x, imageY: y } : p)),
    );
  };

  const updatePendingPointPosition = (x: number, y: number) => {
    setPendingPoint((prev) => (prev ? { ...prev, imageX: x, imageY: y } : null));
  };

  const handleSaveGCP = async () => {
    const incompleteImages = effectiveImages.filter(
      (img) => (pointsByImage[img.filename]?.length || 0) !== REQUIRED_POINTS
    );

    if (incompleteImages.length > 0) {
      alert(
        `Пожалуйста, установите все ${REQUIRED_POINTS} контрольных точек для всех изображений. Осталось изображений: ${incompleteImages.length}`
      );
      return;
    }

    if (projectId) {
      setIsSaving(true);
      setSaveError(null);

      try {
        const payload = effectiveImages.map((img) => ({
          image_filename: img.filename,
          points: pointsByImage[img.filename].map((p) => ({
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
    } else if (imageFilename) {
      const gpcContent = generateGPCContent(completedPoints);
      const element = document.createElement("a");
      element.setAttribute(
        "href",
        "data:text/plain;charset=utf-8," + encodeURIComponent(gpcContent),
      );
      element.setAttribute("download", "calibration.gpc");
      element.style.display = "none";
      document.body.appendChild(element);
      element.click();
      document.body.removeChild(element);
      onComplete();
    }
  };

  const generateGPCContent = (
    calibrationPoints: CalibrationPoint[],
  ): string => {
    let content = "+proj=utm +zone=37 +datum=WGS84\n";
    content += `${imageFilename || "image.jpg"}\n`;
    content += `${calibrationPoints.length}\n`;

    calibrationPoints.forEach((point) => {
      content += `${point.imageX.toFixed(6)} ${point.imageY.toFixed(6)} ${point.lng.toFixed(6)} ${point.lat.toFixed(6)} ${point.altitude.toFixed(2)}\n`;
    });

    return content;
  };

  const getProgressText = () => {
    const totalRequired = effectiveImages.length * REQUIRED_POINTS;
    const totalCompleted = Object.values(pointsByImage).reduce((acc, pts) => acc + pts.length, 0);
    
    if (effectiveImages.length > 1) {
      return `Кадр ${currentImageIndex + 1}/${effectiveImages.length} | Установите 1 точку близкую к центру (Всего: ${totalCompleted}/${totalRequired})`;
    }

    if (!pendingPoint) {
      return `Установите точку близкую к центру`;
    }
    if (!pendingPoint.imageX) {
      return `Выберите точку на изображении`;
    }
    return `Выберите соответствующую точку на карте`;
  };

  return (
    <div className="fixed inset-0 z-[1200] bg-slate-50 flex flex-col">
      <div className="h-full bg-white w-full flex flex-col shadow-xl overflow-hidden">
        {/* Header */}
        <div className="bg-gradient-to-r from-primary to-secondary text-white p-5 flex justify-between items-center shrink-0">
          <div>
            <h2 className="text-xl font-bold">Калибровка системы</h2>
            <p className="text-white/80 text-sm">{getProgressText()}</p>
          </div>
          <Button variant="ghost" onClick={onCancel} className="text-white hover:bg-white/10">
            Отмена
          </Button>
        </div>

        {/* Content - Columns */}
        <div className="flex-1 flex gap-6 p-6 min-h-0">
          {/* Column 0: Image Slider (only if multiple images) */}
          {effectiveImages.length > 1 && (
            <div className="w-24 flex flex-col gap-3 overflow-y-auto pr-2 border-r border-slate-100">
              {effectiveImages.map((img, idx) => (
                <button
                  key={img.filename}
                  onClick={() => {
                    setCurrentImageIndex(idx);
                    setPendingPoint(null);
                    setCurrentMode(null);
                  }}
                  className={cn(
                    "relative aspect-square rounded-lg overflow-hidden border-2 transition-all shrink-0",
                    currentImageIndex === idx ? "border-primary ring-2 ring-primary/20" : "border-transparent hover:border-slate-300"
                  )}
                >
                  <img src={img.url} className="w-full h-full object-cover" alt="" />
                  <div className="absolute bottom-0 left-0 right-0 bg-black/50 text-white text-[8px] py-0.5 text-center">
                    {pointsByImage[img.filename]?.length || 0}/{REQUIRED_POINTS}
                  </div>
                </button>
              ))}
            </div>
          )}

          <div className="flex-1 grid grid-cols-1 lg:grid-cols-12 gap-6 min-h-0">
            {/* Column 1: Image (Col 1-5) */}
            <div className="lg:col-span-5 flex flex-col min-h-0">
              <div className="mb-3 flex justify-between items-center">
                <h3 className="text-sm font-semibold flex items-center gap-2 truncate">
                  📷 {currentImage?.filename || "Изображение"}
                  {currentMode === "image" && (
                    <span className="flex h-2 w-2 rounded-full bg-blue-500 animate-pulse" />
                  )}
                </h3>
                <div className="flex gap-2">
                  <Button 
                      variant={currentMode === "image" ? "default" : "outline"} 
                      size="sm"
                      onClick={() => startNewPoint("image")}
                      disabled={!!pendingPoint || completedPoints.length >= REQUIRED_POINTS}
                  >
                      Добавить
                  </Button>
                </div>
              </div>

              <div
                ref={imageContainerRef}
                onClick={handleImageClick}
                className={cn(
                  "relative rounded-xl overflow-hidden border-2 transition-all aspect-video lg:flex-1 bg-slate-100 shadow-inner",
                  currentMode === "image" ? "border-primary cursor-crosshair ring-4 ring-primary/10" : "border-slate-200"
                )}
              >
                {currentImage && (
                  <img
                    src={currentImage.url}
                    alt="Calibration"
                    className="w-full h-full object-contain pointer-events-none select-none"
                  />
                )}

                {/* Completed points on image - Draggable */}
                <AnimatePresence>
                  {completedPoints.map((point, idx) => (
                    <motion.div
                      key={point.id}
                      drag
                      dragConstraints={imageContainerRef}
                      dragMomentum={false}
                      onDragEnd={(_, info) => {
                        const rect = imageContainerRef.current?.getBoundingClientRect();
                        if (rect) {
                          const x = Math.round(info.point.x - rect.left);
                          const y = Math.round(info.point.y - rect.top);
                          updatePointPosition(point.id, x, y);
                        }
                      }}
                      onClick={(e) => e.stopPropagation()}
                      initial={{ scale: 0, opacity: 0 }}
                      animate={{ scale: 1, opacity: 1, x: 0, y: 0 }}
                      exit={{ scale: 0, opacity: 0 }}
                      transition={{ duration: 0 }}
                      className="absolute w-8 h-8 rounded-full border-2 border-white bg-primary shadow-lg cursor-grab active:cursor-grabbing z-10 flex items-center justify-center text-white text-xs font-bold -ml-4 -mt-4"
                      style={{
                        left: point.imageX,
                        top: point.imageY,
                      }}
                      title={`Точка ${idx + 1}`}
                    >
                      {idx + 1}
                    </motion.div>
                  ))}
                </AnimatePresence>

                {/* Pending point on image */}
                {pendingPoint?.imageX && (
                  <motion.div
                    key={pendingPoint.id}
                    drag
                    dragConstraints={imageContainerRef}
                    dragMomentum={false}
                    onDragEnd={(_, info) => {
                      const rect = imageContainerRef.current?.getBoundingClientRect();
                      if (rect) {
                        const x = Math.round(info.point.x - rect.left);
                        const y = Math.round(info.point.y - rect.top);
                        updatePendingPointPosition(x, y);
                      }
                    }}
                    animate={{ x: 0, y: 0 }}
                    transition={{ duration: 0 }}
                    className="absolute w-8 h-8 rounded-full border-2 border-amber-400 bg-amber-300 shadow-lg animate-pulse z-20 flex items-center justify-center text-amber-900 text-xs font-bold cursor-grab active:cursor-grabbing -ml-4 -mt-4"
                    style={{
                      left: pendingPoint.imageX,
                      top: pendingPoint.imageY,
                    }}
                  >
                    {pointNumber}
                  </motion.div>
                )}
              </div>
            </div>


            {/* Column 2: Map (Col 6-9) */}
            <div className="lg:col-span-4 flex flex-col min-h-0">
              <div className="mb-3 flex justify-between items-center">
                <h3 className="text-sm font-semibold flex items-center gap-2">
                  🗺️ Карта
                  {currentMode === "map" && (
                    <span className="flex h-2 w-2 rounded-full bg-blue-500 animate-pulse" />
                  )}
                </h3>
                <Button 
                  variant={currentMode === "map" ? "default" : "outline"} 
                  size="sm"
                  onClick={() => startNewPoint("map")}
                  disabled={!!pendingPoint || completedPoints.length >= REQUIRED_POINTS}
                >
                  Найти
                </Button>
              </div>

              <div className="relative aspect-video lg:flex-1 rounded-xl overflow-hidden border-2 border-slate-200 bg-slate-100 shadow-inner">
                <MapComponent
                  dronePosition={{ lat: 55.7558, lng: 37.6173 }}
                  followDrone={false}
                  path={completedPoints.map((p) => ({
                    lat: p.lat,
                    lng: p.lng,
                  }))}
                  onMapClick={
                    currentMode === "map" && pendingPoint ? handleMapClick : undefined
                  }
                  selectedPoint={
                    pendingPoint?.lat && pendingPoint?.lng
                      ? { lat: pendingPoint.lat, lng: pendingPoint.lng }
                      : undefined
                  }
                />
              </div>
            </div>

            {/* Column 3: Point List (Col 10-12) */}
            <div className="lg:col-span-3 flex flex-col min-h-0 bg-slate-50 rounded-xl border border-slate-200 p-4">
              <h3 className="text-sm font-semibold mb-3">Список точек</h3>
              
              <Reorder.Group 
                axis="y" 
                values={completedPoints} 
                onReorder={setCompletedPoints}
                className="flex-1 overflow-y-auto space-y-2 pr-1"
              >
                {completedPoints.map((point, idx) => (
                  <Reorder.Item 
                    key={point.id} 
                    value={point}
                    initial={{ opacity: 0, x: 20 }}
                    animate={{ opacity: 1, x: 0 }}
                    className="p-3 rounded-lg bg-white border border-slate-200 shadow-sm flex items-center gap-3 group hover:border-primary/50 transition-colors"
                  >
                    <div className="cursor-grab active:cursor-grabbing text-slate-300 hover:text-slate-500 transition-colors">
                      <GripVertical className="w-4 h-4" />
                    </div>
                    <div className="flex-1 min-w-0">
                      <div className="flex justify-between items-center mb-1">
                        <span className="font-bold text-sm text-slate-700">Точка {completedPoints.indexOf(point) + 1}</span>
                        <button 
                          onClick={() => deletePoint(completedPoints.indexOf(point))}
                          className="text-slate-300 hover:text-red-500 transition-colors"
                        >
                          <Trash2 className="w-3.5 h-3.5" />
                        </button>
                      </div>
                      <div className="text-[10px] font-mono text-slate-500 space-y-0.5">
                        <p>IMG: {point.imageX.toFixed(0)}, {point.imageY.toFixed(0)}</p>
                        <p>GPS: {point.lat.toFixed(5)}, {point.lng.toFixed(5)}</p>
                      </div>
                    </div>
                  </Reorder.Item>
                ))}
                {completedPoints.length === 0 && !pendingPoint && (
                  <div className="flex flex-col items-center justify-center h-32 text-slate-400 text-center">
                    <p className="text-xs">Точки еще не добавлены</p>
                  </div>
                )}
              </Reorder.Group>

              {/* Pending point editor */}
              {pendingPoint && (
                <div className="mt-4 p-3 bg-amber-50 border border-amber-200 rounded-lg animate-in fade-in slide-in-from-bottom-2">
                  <p className="text-xs font-bold text-amber-800 mb-2 uppercase tracking-wider">Новая точка {pointNumber}</p>
                  <div className="space-y-1.5 mb-3 text-[10px] font-mono">
                    <div className="flex justify-between">
                      <span className="text-amber-600">Пиксели:</span>
                      <span className="font-bold">
                        {pendingPoint.imageX ? `${pendingPoint.imageX.toFixed(0)}, ${pendingPoint.imageY?.toFixed(0)}` : "—"}
                      </span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-amber-600">GPS:</span>
                      <span className="font-bold">
                        {pendingPoint.lat ? `${pendingPoint.lat.toFixed(5)}, ${pendingPoint.lng?.toFixed(5)}` : "—"}
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
                    <Button variant="ghost" size="sm" onClick={cancelPoint} className="h-8 text-xs">Отмена</Button>
                    <Button 
                      size="sm" 
                      onClick={() => {
                        const el = document.getElementById("altitude-input") as HTMLInputElement;
                        completePoint(parseFloat(el?.value || "0"));
                      }}
                      disabled={!pendingPoint.imageX || !pendingPoint.lat}
                      className="h-8 text-xs bg-amber-600 hover:bg-amber-700 text-white"
                    >
                      Готово
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
            <p className="text-[10px] text-red-500 mb-3 bg-red-50 p-2 rounded border border-red-100 max-w-md w-full">{saveError}</p>
          )}
          <Button
            onClick={handleSaveGCP}
            disabled={(Object.values(pointsByImage).reduce((acc, pts) => acc + pts.length, 0) < (effectiveImages.length * REQUIRED_POINTS)) || isSaving}
            className="w-full max-w-md gap-2 h-12 font-bold text-sm shadow-lg shadow-primary/20"
          >
            {isSaving ? (
              <div className="animate-spin rounded-full h-4 w-4 border-2 border-white/30 border-t-white" />
            ) : (
              <Save className="w-4 h-4" />
            )}
            {isSaving ? "Сохранение..." : "Завершить калибровку"}
          </Button>
          <p className="text-[10px] text-center text-slate-400 mt-3">
            Необходимо установить по 1 точке для каждого изображения (минимум 5 изображений)
          </p>
        </div>
      </div>
    </div>
  );
}
