import { useEffect, useRef, useState } from "react";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import { cn } from "@/lib/utils";
import {
  OFFLINE_STYLE,
  styleFor,
  useInternetStatus,
  type MapLayer,
} from "@/lib/mapBasemap";

interface MapComponentProps {
  dronePosition?: {
    lat: number;
    lng: number;
  };
  path?: Array<{ lat: number; lng: number }>;
  onMapClick?: (lat: number, lng: number) => void;
  selectedPoint?: { lat: number; lng: number };
  showMarkers?: boolean;
  followDrone?: boolean;
}

const DRONE_ICON_SVG =
  "data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMzIiIGhlaWdodD0iMzIiIHZpZXdCb3g9IjAgMCAzMiAzMiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPGNpcmNsZSBjeD0iMTYiIGN5PSIxNiIgcj0iMTQiIGZpbGw9IiMwMDdhZjgiIHN0cm9rZT0id2hpdGUiIHN0cm9rZS13aWR0aD0iMiIvPgo8cGF0aCBkPSJNMTYgOEwxOSAxNEgxM0wxNiA4WiIgZmlsbD0id2hpdGUiLz4KPHBhdGggZD0iTTI0IDE2TDE4IDE5VjEzTDI0IDE2WiIgZmlsbD0id2hpdGUiLz4KPHBhdGggZD0iTTggMTZMMTQgMTlWMTNMOCAxNloiIGZpbGw9IndoaXRlIi8+CjxwYXRoIGQ9Ik0xNiAyNEwxMyAyMFYyNkwxNiAyNFoiIGZpbGw9IndoaXRlIi8+Cjwvc3ZnPg==";

const makeDroneElement = () => {
  const el = document.createElement("div");
  el.style.width = "32px";
  el.style.height = "32px";
  el.innerHTML = `<img src="${DRONE_ICON_SVG}" width="32" height="32" style="display:block" />`;
  return el;
};

const makeNumberedElement = (idx: number) => {
  const el = document.createElement("div");
  el.style.width = "24px";
  el.style.height = "24px";
  el.innerHTML =
    '<div class="w-6 h-6 rounded-full border-2 border-white bg-primary shadow-lg flex items-center justify-center text-white text-[10px] font-bold">' +
    (idx + 1) +
    "</div>";
  return el;
};

const makeSelectedElement = () => {
  const el = document.createElement("div");
  el.style.width = "32px";
  el.style.height = "32px";
  el.innerHTML =
    '<div class="w-8 h-8 rounded-full border-2 border-amber-400 bg-amber-300 shadow-lg animate-pulse"></div>';
  return el;
};

const pathToGeoJSON = (pts: Array<{ lat: number; lng: number }>) => ({
  type: "FeatureCollection" as const,
  features: [
    {
      type: "Feature" as const,
      geometry: {
        type: "LineString" as const,
        coordinates: pts.map((p) => [p.lng, p.lat]),
      },
      properties: {},
    },
  ],
});

export function MapComponent({
  dronePosition = { lat: 55.7558, lng: 37.6173 }, // Default: Moscow
  path = [],
  onMapClick,
  selectedPoint,
  showMarkers = true,
  followDrone = true,
}: MapComponentProps) {
  const mapId = useRef(`map-${Math.random().toString(36).substr(2, 9)}`);
  const mapRef = useRef<maplibregl.Map | null>(null);
  const droneMarkerRef = useRef<maplibregl.Marker | null>(null);
  const pathMarkersRef = useRef<maplibregl.Marker[]>([]);
  const selectedMarkerRef = useRef<maplibregl.Marker | null>(null);
  const clickHandlerRef = useRef<((e: any) => void) | null>(null);

  // Keep the latest props in refs so the (mount-only) map handlers can read them.
  const pathRef = useRef(path);
  pathRef.current = path;
  const showMarkersRef = useRef(showMarkers);
  showMarkersRef.current = showMarkers;
  const dronePosRef = useRef(dronePosition);
  dronePosRef.current = dronePosition;

  // Connectivity + selected online layer.
  const online = useInternetStatus();
  const [layer, setLayer] = useState<MapLayer>("vector");

  // (Re)create the drone-path source/layer and push the latest path data.
  const ensurePathLayer = (map: maplibregl.Map) => {
    if (!map.isStyleLoaded()) return;
    if (!map.getSource("drone-path")) {
      map.addSource("drone-path", {
        type: "geojson",
        data: pathToGeoJSON([]),
      });
      map.addLayer({
        id: "drone-path-line",
        type: "line",
        source: "drone-path",
        paint: {
          "line-color": "#007af8",
          "line-width": 3,
          "line-opacity": 0.7,
          "line-dasharray": [5, 5],
        },
      });
    }
    (map.getSource("drone-path") as maplibregl.GeoJSONSource).setData(
      pathToGeoJSON(pathRef.current),
    );
  };

  // Create the map once, starting from the offline style (safe default).
  useEffect(() => {
    if (mapRef.current) return;

    const map = new maplibregl.Map({
      container: mapId.current,
      style: styleFor(online, layer),
      center: [dronePosition.lng, dronePosition.lat],
      zoom: 17,
    });

    map.addControl(new maplibregl.NavigationControl(), "bottom-right");

    // Keep the UI usable if some tiles are missing.
    map.on("error", (e) => {
      console.warn("MapLibre error:", e && e.error ? e.error.message : e);
    });

    // After any style (re)load, re-add the drone-path layer (it lives in the
    // style and is wiped by setStyle) and re-apply the latest path data.
    map.on("style.load", () => ensurePathLayer(map));

    const droneMarker = new maplibregl.Marker({ element: makeDroneElement() })
      .setLngLat([dronePosition.lng, dronePosition.lat])
      .setPopup(
        new maplibregl.Popup({ offset: 16 }).setHTML(
          "<b>📍 Позиция дрона</b>",
        ),
      )
      .addTo(map);
    droneMarkerRef.current = droneMarker;

    mapRef.current = map;

    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
        droneMarkerRef.current = null;
        selectedMarkerRef.current = null;
        pathMarkersRef.current = [];
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Switch the basemap whenever connectivity or the chosen online layer changes.
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;
    map.setStyle(styleFor(online, layer));
  }, [online, layer]);

  // Handle onMapClick changes
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    if (clickHandlerRef.current) map.off("click", clickHandlerRef.current);
    if (onMapClick) {
      const handler = (e: any) => onMapClick(e.lngLat.lat, e.lngLat.lng);
      map.on("click", handler);
      clickHandlerRef.current = handler;
    } else {
      clickHandlerRef.current = null;
    }
  }, [onMapClick]);

  // Update drone position
  useEffect(() => {
    const map = mapRef.current;
    const marker = droneMarkerRef.current;
    if (marker && map) {
      marker.setLngLat([dronePosition.lng, dronePosition.lat]);
      if (followDrone) {
        map.easeTo({ center: [dronePosition.lng, dronePosition.lat] });
      }
    }
  }, [dronePosition, followDrone]);

  // Update path (numbered markers + line) and fit bounds when it grows.
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    // Remove old numbered markers
    pathMarkersRef.current.forEach((m) => m.remove());
    pathMarkersRef.current = [];

    if (showMarkers) {
      path.forEach((p, idx) => {
        const marker = new maplibregl.Marker({ element: makeNumberedElement(idx) })
          .setLngLat([p.lng, p.lat])
          .addTo(map);
        pathMarkersRef.current.push(marker);
      });
    }

    if (map.isStyleLoaded()) ensurePathLayer(map);
    else map.once("style.load", () => ensurePathLayer(map));

    if (path.length > 1) {
      const bounds = new maplibregl.LngLatBounds();
      path.forEach((p) => bounds.extend([p.lng, p.lat]));
      bounds.extend([dronePosition.lng, dronePosition.lat]);
      map.fitBounds(bounds, { padding: 50 });
    }
  }, [path, showMarkers, dronePosition]);

  // Update selected point marker
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    if (selectedPoint) {
      if (selectedMarkerRef.current) {
        selectedMarkerRef.current.setLngLat([
          selectedPoint.lng,
          selectedPoint.lat,
        ]);
      } else {
        const marker = new maplibregl.Marker({
          element: makeSelectedElement(),
        })
          .setLngLat([selectedPoint.lng, selectedPoint.lat])
          .addTo(map);
        selectedMarkerRef.current = marker;
      }
    } else if (selectedMarkerRef.current) {
      selectedMarkerRef.current.remove();
      selectedMarkerRef.current = null;
    }
  }, [selectedPoint]);

  return (
    <div style={{ position: "relative", width: "100%", height: "100%" }}>
      <div
        id={mapId.current}
        style={{
          width: "100%",
          height: "100%",
          borderRadius: "0.5rem",
        }}
      />

      {/* Online basemap switcher (vector <-> satellite) */}
      {online === true && (
        <div className="absolute top-3 left-3 z-10 flex overflow-hidden rounded-md border border-black/10 shadow-sm">
          <button
            type="button"
            onClick={() => setLayer("vector")}
            className={cn(
              "px-3 py-1.5 text-xs font-medium transition-colors",
              layer === "vector"
                ? "bg-primary text-white"
                : "bg-white/90 text-slate-700 hover:bg-white",
            )}
          >
            Схема
          </button>
          <button
            type="button"
            onClick={() => setLayer("satellite")}
            className={cn(
              "px-3 py-1.5 text-xs font-medium transition-colors border-l border-black/10",
              layer === "satellite"
                ? "bg-primary text-white"
                : "bg-white/90 text-slate-700 hover:bg-white",
            )}
          >
            Спутник
          </button>
        </div>
      )}

      {/* Offline indicator */}
      {online === false && (
        <div className="absolute top-3 left-3 z-10 flex items-center gap-1.5 rounded-md border border-amber-300 bg-amber-50/95 px-2.5 py-1.5 text-xs font-medium text-amber-700 shadow-sm">
          <span className="h-2 w-2 rounded-full bg-amber-500" />
          Офлайн-карта
        </div>
      )}
    </div>
  );
}
