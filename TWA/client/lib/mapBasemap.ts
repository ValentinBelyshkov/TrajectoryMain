import { useEffect, useState } from "react";

export type MapLayer = "vector" | "satellite";

// Reuse the exact same offline vector tiles that the :9000 relay serves.
// CORS is already allowed by the relay tile endpoint (Access-Control-Allow-Origin: *).
export const OFFLINE_TILES_URL =
  "http://192.168.0.1:9000/api/v1/map/tiles/{z}/{x}/{y}.pbf";

// MapLibre style that renders the offline vector tiles (same as the relay's
// offline-style.json), so TWA shows the identical offline map as :9000.
export const OFFLINE_STYLE: any = {
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

// Free, no-API-key online vector basemap (OpenFreeMap "liberty" style).
export const VECTOR_ONLINE_STYLE =
  "https://tiles.openfreemap.org/styles/liberty";

// Free, no-API-key online satellite imagery (Esri World Imagery raster tiles).
export const SATELLITE_STYLE: any = {
  version: 8,
  sources: {
    "esri-imagery": {
      type: "raster",
      tiles: [
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      ],
      tileSize: 256,
      maxzoom: 19,
      attribution:
        "Imagery &copy; Esri, Maxar, Earthstar Geographics, and the GIS User Community",
    },
  },
  layers: [
    { id: "esri-imagery", type: "raster", source: "esri-imagery" },
  ],
};

// Pick the MapLibre style for the current connectivity + layer choice.
// When offline (or while still probing) we always fall back to the offline tiles.
export const styleFor = (isOnline: boolean | null, layer: MapLayer): any => {
  if (!isOnline) return OFFLINE_STYLE;
  return layer === "satellite" ? SATELLITE_STYLE : VECTOR_ONLINE_STYLE;
};

// Probe a few public endpoints to decide whether the browser has internet.
// Uses no-cors so a network failure rejects (offline) while a successful
// response resolves (online). First probe to succeed wins.
const CONNECTIVITY_PROBES = [
  "https://www.gstatic.com/generate_204",
  "https://tile.openstreetmap.org/0/0/0.png",
  "https://tiles.openfreemap.org/styles/liberty",
];

async function probeInternet(timeoutMs = 4000): Promise<boolean> {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs);
  try {
    for (const url of CONNECTIVITY_PROBES) {
      try {
        await fetch(url, {
          mode: "no-cors",
          cache: "no-store",
          signal: controller.signal,
        });
        return true;
      } catch {
        // this probe failed (offline / blocked) – try the next one
      }
    }
    return false;
  } finally {
    clearTimeout(timer);
  }
}

// Reactively track internet connectivity:
//  - probes on mount, on browser online/offline events, and every 30s
//  - returns null while still probing, true when online, false when offline
export function useInternetStatus(): boolean | null {
  const [online, setOnline] = useState<boolean | null>(null);

  useEffect(() => {
    let cancelled = false;

    const check = async () => {
      const hasInternet = await probeInternet();
      if (!cancelled) setOnline(hasInternet);
    };

    check();
    const interval = setInterval(check, 30000);
    window.addEventListener("online", check);
    window.addEventListener("offline", check);

    return () => {
      cancelled = true;
      clearInterval(interval);
      window.removeEventListener("online", check);
      window.removeEventListener("offline", check);
    };
  }, []);

  return online;
}
