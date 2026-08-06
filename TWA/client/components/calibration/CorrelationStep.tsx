import { useState, useEffect, useRef } from "react";
import { useQuery } from "@tanstack/react-query";
import maplibregl from "maplibre-gl";
import { MapPin } from "lucide-react";
import { procframe } from "@/lib/api";

interface CorrelationStepProps {
  sessionId: string;
  projectId: string;
  frames: Array<{ frame: string; pose_file?: string; pose?: { x: number; y: number; z: number }; timestamp: number }>;
  onCompute: (points: any[]) => void;
  onBack: () => void;
  sessionStatus?: string;
}

interface CorrelationPoint {
  frame_idx: number;
  pixel_x: number;
  pixel_y: number;
  lat: number;
  lon: number;
  alt: number;
}

export function CorrelationStep({ sessionId, projectId, frames, onCompute, onBack, sessionStatus }: CorrelationStepProps) {
  const [selectedFrame, setSelectedFrame] = useState<number | null>(null);
  const [pixelPoint, setPixelPoint] = useState<{ x: number; y: number } | null>(null);
  const [gpsPoint, setGpsPoint] = useState<{ lat: number; lon: number; alt: number } | null>(null);
  const [points, setPoints] = useState<CorrelationPoint[]>([]);
  const [map, setMap] = useState<maplibregl.Map | null>(null);
  const [isInitialLoad, setIsInitialLoad] = useState(true);
  const mapContainerRef = useRef<HTMLDivElement | null>(null);
  const markerRef = useRef<maplibregl.Marker | null>(null);

  const { data: procframeData = [] } = useQuery({
    queryKey: ["procframe-frames", projectId],
    queryFn: () => procframe(projectId),
    enabled: !!projectId && frames.length === 0,
  });

  const procframeFrames = Array.isArray(procframeData)
    ? procframeData
    : (procframeData as any)?.frames || [];

  const displayFrames =
    frames.length > 0
      ? frames.map((f) => ({ frame: f.frame, pose_file: f.pose_file, pose: f.pose, timestamp: f.timestamp }))
      : procframeFrames.map((f) => ({ frame: f.filename, url: f.url }));

  useEffect(() => {
    if (displayFrames.length > 0) {
      setIsInitialLoad(false);
    }
  }, [displayFrames]);

  const initMap = () => {
    if (!mapContainerRef.current || map) return;
    try {
      const m = new maplibregl.Map({
        container: mapContainerRef.current,
        style: ({
          version: 8,
          name: "Offline Central Russia",
          center: [38.634, 55.492],
          zoom: 17,
          sources: {
            "local-tiles": {
              type: "vector",
              tiles: ["http://192.168.0.1:9000/api/v1/map/tiles/{z}/{x}/{y}.pbf"],
              maxzoom: 14,
            },
          },
          layers: [
            { id: "background", type: "background", paint: { "background-color": "#f2efe9" } },
            { id: "road", type: "line", source: "local-tiles", "source-layer": "transportation", filter: ["has", "class"], paint: { "line-color": "#ffffff", "line-width": { base: 1.5, stops: [[5, 1], [14, 8]] }, "line-opacity": 0.9 } },
          ],
        }) as any,
      });
      m.addControl(new maplibregl.NavigationControl());
      m.on("click", (e: any) => {
        setGpsPoint({ lat: e.lngLat.lat, lon: e.lngLat.lng, alt: 100 });
        if (markerRef.current) markerRef.current.remove();
        markerRef.current = new maplibregl.Marker({ color: "#3b82f6" }).setLngLat([e.lngLat.lng, e.lngLat.lat]).addTo(m);
      });
      setMap(m);
    } catch (e) {
      console.error("Map init error:", e);
    }
  };

  useEffect(() => {
    if (selectedFrame !== null) {
      setTimeout(initMap, 100);
    }
    return () => {
      if (markerRef.current) markerRef.current.remove();
    };
  }, [selectedFrame]);

  const handleFrameClick = (e: React.MouseEvent<HTMLDivElement>, frameIdx: number) => {
    setSelectedFrame(frameIdx);
  };

  const confirmPoint = () => {
    if (selectedFrame === null || !pixelPoint || !gpsPoint) return;
    const selectedFrameData = displayFrames[selectedFrame];
    if (selectedFrameData && !selectedFrameData.pose) {
      alert("Для выбранного кадра отсутствуют данные позы. Выберите другой кадр.");
      return;
    }
    const newPoint: CorrelationPoint = {
      frame_idx: selectedFrame,
      pixel_x: pixelPoint.x,
      pixel_y: pixelPoint.y,
      ...gpsPoint,
    };
    setPoints((prev) => [...prev, newPoint]);
    setPixelPoint(null);
    setGpsPoint(null);
  };

  const handleCompute = () => {
    if (points.length !== 5) return;
    onCompute(points);
  };

  return (
    <div className="flex flex-col h-full">
      <div className="flex-1 overflow-auto p-6">
        <div className="max-w-6xl mx-auto space-y-6">
          <div className="text-center">
            <h3 className="text-2xl font-bold text-foreground mb-2">Соотнесение (5 точек)</h3>
            <p className="text-muted-foreground">Выберите точки на кадрах и на карте</p>
          </div>

          <div className="flex gap-2 flex-wrap justify-center">
            {isInitialLoad && displayFrames.length === 0 && (
              <div className="col-span-full text-center text-muted-foreground py-8">
                Загрузка кадров...
              </div>
            )}
            {!isInitialLoad && displayFrames.length === 0 && sessionStatus === "error" && (
              <div className="col-span-full text-center text-red-500 py-8">
                Ошибка обработки: не найдено кадров с валидными позами. Карта не создана.
              </div>
            )}
            {!isInitialLoad && displayFrames.length === 0 && sessionStatus !== "error" && (
              <div className="col-span-full text-center text-muted-foreground py-8">
                Кадры не найдены
              </div>
            )}
            {displayFrames.map((frame, idx) => (
              <div
                key={idx}
                onClick={(e) => handleFrameClick(e, idx)}
                className="w-24 h-24 border-2 border-border rounded-lg overflow-hidden cursor-crosshair hover:border-blue-500 transition-colors relative"
              >
                <img
                  src={`/api/projects/${projectId}/procframe/${frame.frame}`}
                  alt={`Frame ${idx + 1}`}
                  className="w-full h-full object-cover"
                />
                {points.some((p) => p.frame_idx === idx) && (
                  <div className="absolute top-1 right-1 bg-green-500 text-white text-xs rounded-full w-5 h-5 flex items-center justify-center">
                    ✓
                  </div>
                )}
              </div>
            ))}
          </div>

          {selectedFrame !== null && (
            <div className="border border-border rounded-lg p-4 bg-white">
              <h4 className="font-bold mb-3">Кадр {selectedFrame + 1}</h4>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div className="relative">
                  <img
                    src={`/api/projects/${projectId}/procframe/${displayFrames[selectedFrame].frame}`}
                    alt={`Frame ${selectedFrame}`}
                    className="w-full aspect-video object-contain border border-border rounded cursor-crosshair"
                    onClick={(e) => {
                      const rect = e.currentTarget.getBoundingClientRect();
                      const x = ((e.clientX - rect.left) / rect.width) * 100;
                      const y = ((e.clientY - rect.top) / rect.height) * 100;
                      setPixelPoint({ x, y });
                    }}
                  />
                  {pixelPoint && (
                    <div
                      className="absolute w-3 h-3 bg-red-500 border-2 border-white rounded-full transform -translate-x-1/2 -translate-y-1/2 pointer-events-none"
                      style={{
                        left: `${pixelPoint.x}%`,
                        top: `${pixelPoint.y}%`,
                      }}
                    />
                  )}
                </div>
                <div>
                  <div ref={mapContainerRef} className="w-full h-[300px] border border-border rounded" />
                  <p className="text-xs text-muted-foreground mt-1">
                    Кликните на карте для GPS. {gpsPoint ? `${gpsPoint.lat.toFixed(6)}, ${gpsPoint.lon.toFixed(6)}` : "Не выбрано"}
                  </p>
                </div>
              </div>
              <div className="mt-4 flex gap-2">
                <button
                  onClick={confirmPoint}
                  disabled={!pixelPoint || !gpsPoint || (selectedFrame !== null && !displayFrames[selectedFrame]?.pose)}
                  className="px-4 py-2 bg-primary text-white rounded-lg font-bold hover:bg-primary/90 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
                >
                  ✓ Подтвердить точку
                </button>
                <button onClick={() => { setSelectedFrame(null); setPixelPoint(null); setGpsPoint(null); }} className="px-4 py-2 border border-border rounded-lg hover:bg-slate-50 transition-colors">
                  Отмена
                </button>
              </div>
            </div>
          )}

          {points.length > 0 && (
            <div className="border border-border rounded-lg overflow-hidden">
              <table className="w-full text-sm">
                <thead>
                  <tr className="bg-slate-50 text-left text-muted-foreground">
                    <th className="p-2">#</th>
                    <th className="p-2">Frame</th>
                    <th className="p-2">Pixel</th>
                    <th className="p-2">GPS</th>
                  </tr>
                </thead>
                <tbody>
                  {points.map((p, i) => (
                    <tr key={i} className="border-t border-border">
                      <td className="p-2">{i + 1}</td>
                      <td className="p-2">Frame {p.frame_idx + 1}</td>
                      <td className="p-2">({p.pixel_x.toFixed(1)}%, {p.pixel_y.toFixed(1)}%)</td>
                      <td className="p-2">{p.lat.toFixed(6)}, {p.lon.toFixed(6)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}

          <div className="flex justify-between">
            <button onClick={onBack} className="px-4 py-2 border border-border rounded-lg hover:bg-slate-50 transition-colors">
              ← Назад
            </button>
            <button
              onClick={handleCompute}
              disabled={points.length !== 5}
              className="px-6 py-2 bg-purple-600 text-white rounded-lg font-bold hover:bg-purple-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
            >
              Соотнести ({points.length}/5)
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}
