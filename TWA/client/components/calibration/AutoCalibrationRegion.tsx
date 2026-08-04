import { useState, useEffect, useRef } from "react";
import { MapComponent } from "@/components/MapComponent";
import { Button } from "@/components/ui/button";
import { Alert, AlertDescription, AlertTitle } from "@/components/ui/alert";
import { AlertCircle, CheckCircle2, MapPin, ZoomIn, ZoomOut } from "lucide-react";
import type { AutoCalibrationRegion } from "@/lib/api";

interface AutoCalibrationRegionProps {
  onConfirm: (region: AutoCalibrationRegion) => void;
  onBack: () => void;
  dronePosition: { lat: number; lng: number };
  error?: string | null;
  progress?: "idle" | "downloading" | "matching" | "success" | "error";
  message?: string | null;
}

export function AutoCalibrationRegion({
  onConfirm,
  onBack,
  dronePosition,
  error,
  progress,
  message,
}: AutoCalibrationRegionProps) {
  const [selectionStart, setSelectionStart] = useState<{ lat: number; lng: number } | null>(null);
  const [selectionEnd, setSelectionEnd] = useState<{ lat: number; lng: number } | null>(null);
  const [zoom, setZoom] = useState(15);
  const [mapCenter, setMapCenter] = useState(dronePosition);
  const isSelectingRef = useRef(false);

  const handleMapClick = (lat: number, lng: number) => {
    if (progress === "downloading" || progress === "matching") return;
    
    if (!selectionStart) {
      setSelectionStart({ lat, lng });
      setSelectionEnd({ lat, lng });
    } else if (!selectionEnd || (selectionStart.lat === selectionEnd.lat && selectionStart.lng === selectionEnd.lng)) {
      setSelectionEnd({ lat, lng });
    } else {
      // Reset and start new selection
      setSelectionStart({ lat, lng });
      setSelectionEnd({ lat, lng });
    }
  };

  const selectionRect = selectionStart && selectionEnd ? {
    sw: {
      lat: Math.min(selectionStart.lat, selectionEnd.lat),
      lng: Math.min(selectionStart.lng, selectionEnd.lng),
    },
    ne: {
      lat: Math.max(selectionStart.lat, selectionEnd.lat),
      lng: Math.max(selectionStart.lng, selectionEnd.lng),
    },
  } : null;

  const handleConfirm = () => {
    if (!selectionRect) return;
    
    onConfirm({
      lat1: selectionRect.sw.lat,
      lng1: selectionRect.sw.lng,
      lat2: selectionRect.ne.lat,
      lng2: selectionRect.ne.lng,
      zoom: zoom,
    });
  };

  const handleZoomIn = () => setZoom(z => Math.min(z + 1, 19));
  const handleZoomOut = () => setZoom(z => Math.max(z - 1, 1));

  return (
    <div className="flex flex-col h-full">
      <div className="bg-gradient-to-r from-blue-600 to-blue-800 text-white p-6 shrink-0">
        <h2 className="text-2xl font-bold mb-2">Автоматическая калибровка</h2>
        <p className="text-blue-100">
          Выберите регион на карте, где будет проходить полёт
        </p>
      </div>

      <div className="flex-1 relative min-h-0">
        {/* Map controls */}
        <div className="absolute top-4 left-4 z-10 flex flex-col gap-2">
          <button
            onClick={handleZoomIn}
            className="w-10 h-10 bg-white rounded-lg shadow-md flex items-center justify-center hover:bg-slate-50 transition-colors"
          >
            <ZoomIn className="w-5 h-5 text-slate-600" />
          </button>
          <button
            onClick={handleZoomOut}
            className="w-10 h-10 bg-white rounded-lg shadow-md flex items-center justify-center hover:bg-slate-50 transition-colors"
          >
            <ZoomOut className="w-5 h-5 text-slate-600" />
          </button>
        </div>

        {/* Selection info panel */}
        <div className="absolute top-4 right-4 z-10 bg-white rounded-lg shadow-md p-4 min-w-64">
          <div className="flex items-center gap-2 mb-2">
            <MapPin className="w-5 h-5 text-blue-600" />
            <span className="font-semibold">Выбор региона</span>
          </div>
          
          {!selectionStart ? (
            <p className="text-sm text-muted-foreground">
              Кликните на карте, чтобы выбрать первую точку
            </p>
          ) : (
            <div className="space-y-2 text-sm">
              <div>
                <span className="text-muted-foreground">Точка 1:</span>
                <p className="font-mono">
                  {selectionStart.lat.toFixed(6)}, {selectionStart.lng.toFixed(6)}
                </p>
              </div>
              
              {selectionEnd && (
                <div>
                  <span className="text-muted-foreground">Точка 2:</span>
                  <p className="font-mono">
                    {selectionEnd.lat.toFixed(6)}, {selectionEnd.lng.toFixed(6)}
                  </p>
                </div>
              )}
              
              {selectionRect && (
                <div className="pt-2 border-t">
                  <div className="grid grid-cols-2 gap-2">
                    <div>
                      <span className="text-muted-foreground">Широта:</span>
                      <p className="font-mono text-xs">
                        {selectionRect.sw.lat.toFixed(4)} - {selectionRect.ne.lat.toFixed(4)}
                      </p>
                    </div>
                    <div>
                      <span className="text-muted-foreground">Долгота:</span>
                      <p className="font-mono text-xs">
                        {selectionRect.sw.lng.toFixed(4)} - {selectionRect.ne.lng.toFixed(4)}
                      </p>
                    </div>
                  </div>
                </div>
              )}
            </div>
          )}
        </div>

        {/* Map */}
        <MapComponent
          dronePosition={mapCenter}
          onMapClick={handleMapClick}
          selectedPoint={selectionEnd || undefined}
          showMarkers={true}
          followDrone={false}
        />

        {/* Selection overlay instructions */}
        {selectionStart && !selectionEnd && (
          <div className="absolute bottom-24 left-1/2 transform -translate-x-1/2 bg-blue-600 text-white px-6 py-3 rounded-lg shadow-lg z-10">
            Кликните для выбора второй точки (противоположный угол)
          </div>
        )}
      </div>

      {/* Error/Status messages */}
      {error && (
        <Alert variant="destructive" className="m-4 shrink-0">
          <AlertCircle className="h-4 w-4" />
          <AlertTitle>Ошибка</AlertTitle>
          <AlertDescription>{error}</AlertDescription>
        </Alert>
      )}

      {progress === "downloading" && message && (
        <Alert className="m-4 shrink-0 border-blue-200 bg-blue-50">
          <div className="animate-spin rounded-full h-4 w-4 border-2 border-blue-600 border-t-transparent" />
          <AlertTitle>Загрузка карты</AlertTitle>
          <AlertDescription>{message}</AlertDescription>
        </Alert>
      )}

      <div className="p-6 border-t bg-slate-50 shrink-0">
        <div className="flex items-center justify-center gap-4">
          <Button variant="outline" onClick={onBack}>
            Назад
          </Button>
          <Button
            onClick={handleConfirm}
            disabled={!selectionRect || progress === "downloading" || progress === "matching"}
            className="gap-2"
          >
            <CheckCircle2 className="w-4 h-4" />
            Обработать
          </Button>
        </div>
      </div>
    </div>
  );
}
