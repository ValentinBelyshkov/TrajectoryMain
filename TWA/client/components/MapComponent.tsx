import { useEffect, useRef } from "react";
import maplibregl from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";

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

// Reuse the exact same offline vector tiles that the :9000 relay serves.
// CORS is already allowed by the relay tile endpoint (Access-Control-Allow-Origin: *).
const OFFLINE_TILES_URL =
  "http://192.168.0.1:9000/api/v1/map/tiles/{z}/{x}/{y}.pbf";

// MapLibre style that renders the offline vector tiles (same as the relay's
// offline-style.json), so TWA shows the identical offline map as :9000.
const OFFLINE_STYLE: any = {
  version: 8,
  name: "Offline Central Russia",
  center: [38.634, 55.492],
  zoom: 7,
  sources: {
    "local-tiles": {
      type: "vector",
      tiles: [OFFLINE_TILES_URL],
      maxzoom: 14,
    },
  },
  layers: [
    { id: "background", type: "background", paint: { "background-color": "#f2efe9" } },
    { id: "water", type: "fill", source: "local-tiles", "source-layer": "water", paint: { "fill-color": "#aadaff" } },
    { id: "waterway", type: "line", source: "local-tiles", "source-layer": "waterway", paint: { "line-color": "#aadaff", "line-width": { base: 1.2, stops: [[8, 0.8], [14, 2]] } } },
    { id: "landcover", type: "fill", source: "local-tiles", "source-layer": "landcover", paint: { "fill-color": "#eef5e6" } },
    { id: "landuse", type: "fill", source: "local-tiles", "source-layer": "landuse", paint: { "fill-color": "#dde8c4" } },
    { id: "park", type: "fill", source: "local-tiles", "source-layer": "park", paint: { "fill-color": "#cde2a8" } },
    { id: "aeroway", type: "fill", source: "local-tiles", "source-layer": "aeroway", filter: ["==", "class", "aerodrome"], paint: { "fill-color": "#d9d0c9", "fill-opacity": 0.6 } },
    { id: "building", type: "fill", source: "local-tiles", "source-layer": "building", paint: { "fill-color": "#d9d0c9", "fill-opacity": 0.9 } },
    { id: "road", type: "line", source: "local-tiles", "source-layer": "transportation", filter: ["has", "class"], paint: { "line-color": "#ffffff", "line-width": { base: 1.5, stops: [[5, 1], [14, 8]] }, "line-opacity": 0.9 } },
    { id: "road-case", type: "line", source: "local-tiles", "source-layer": "transportation", filter: ["has", "class"], paint: { "line-color": "#b9b0a8", "line-width": { base: 1.5, stops: [[5, 1.2], [14, 9]] }, "line-opacity": 0.6 } },
    { id: "boundary", type: "line", source: "local-tiles", "source-layer": "boundary", paint: { "line-color": "#f85149", "line-width": { base: 1.2, stops: [[6, 1], [12, 2.5]] }, "line-dasharray": [3, 2] } },
    { id: "poi", type: "circle", source: "local-tiles", "source-layer": "poi", paint: { "circle-radius": { base: 1.3, stops: [[10, 2], [14, 5]] }, "circle-color": "#bc8cff", "circle-stroke-width": 1, "circle-stroke-color": "#ffffff" } },
  ],
};

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

  useEffect(() => {
    if (mapRef.current) return;

    const map = new maplibregl.Map({
      container: mapId.current,
      style: OFFLINE_STYLE,
      center: [dronePosition.lng, dronePosition.lat],
      zoom: 17,
    });

    map.addControl(new maplibregl.NavigationControl(), "bottom-right");

    // Keep the UI usable if some offline tiles are missing.
    map.on("error", (e) => {
      console.warn("MapLibre error:", e && e.error ? e.error.message : e);
    });

    const droneMarker = new maplibregl.Marker({ element: makeDroneElement() })
      .setLngLat([dronePosition.lng, dronePosition.lat])
      .setPopup(new maplibregl.Popup({ offset: 16 }).setHTML("<b>📍 Позиция дрона</b>"))
      .addTo(map);
    droneMarkerRef.current = droneMarker;

    const applyPath = () => {
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
        pathToGeoJSON(path),
      );
      if (path.length > 1) {
        const bounds = new maplibregl.LngLatBounds();
        path.forEach((p) => bounds.extend([p.lng, p.lat]));
        bounds.extend([dronePosition.lng, dronePosition.lat]);
        map.fitBounds(bounds, { padding: 50 });
      }
    };

    map.on("load", applyPath);
    if (map.isStyleLoaded()) applyPath();

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

  // Update path
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

    const apply = () => {
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
        pathToGeoJSON(path),
      );
    };

    if (map.isStyleLoaded()) apply();
    else map.once("load", apply);
  }, [path, showMarkers]);

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
    <div
      id={mapId.current}
      style={{
        width: "100%",
        height: "100%",
        borderRadius: "0.5rem",
      }}
    />
  );
}
