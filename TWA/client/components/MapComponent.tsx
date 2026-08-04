import { useEffect, useRef } from "react";
import L from "leaflet";

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

// Fix for default marker icons in Leaflet - moved inside component to avoid SSR issues
const fixLeafletIcons = () => {
  if (typeof window === "undefined") return;
  try {
    const proto = L.Icon.Default.prototype as any;
    if (proto._getIconUrl) {
      delete proto._getIconUrl;
      L.Icon.Default.mergeOptions({
        iconRetinaUrl:
          "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon-2x.png",
        iconUrl:
          "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-icon.png",
        shadowUrl:
          "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/images/marker-shadow.png",
      });
    }
  } catch (e) {
    console.warn("Leaflet icon fix failed:", e);
  }
};

export function MapComponent({
  dronePosition = { lat: 55.7558, lng: 37.6173 }, // Default: Moscow
  path = [],
  onMapClick,
  selectedPoint,
  showMarkers = true,
  followDrone = true,
}: MapComponentProps) {
  const mapId = useRef(`map-${Math.random().toString(36).substr(2, 9)}`);
  const mapRef = useRef<L.Map | null>(null);
  const droneMarkerRef = useRef<L.Marker | null>(null);
  const pathPolylineRef = useRef<L.Polyline | null>(null);
  const selectedPointMarkerRef = useRef<L.Marker | null>(null);
  const pathMarkersRef = useRef<L.Marker[]>([]);

  useEffect(() => {
    // Apply Leaflet icon fix
    fixLeafletIcons();

    // Load Leaflet CSS dynamically
    if (
      typeof document !== "undefined" &&
      !document.getElementById("leaflet-css")
    ) {
      const link = document.createElement("link");
      link.id = "leaflet-css";
      link.rel = "stylesheet";
      link.href =
        "https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css";
      document.head.appendChild(link);
    }
  }, []);

  useEffect(() => {
    if (mapRef.current) return;

    // Initialize map
    const map = L.map(mapId.current, {
      zoomControl: false // We'll add it later or keep default, but usually custom positioning is better
    }).setView(
      [dronePosition.lat, dronePosition.lng],
      17,
    );

    L.control.zoom({ position: 'bottomright' }).addTo(map);

    // Add OpenStreetMap tile layer
    const osm = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution:
        '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      maxZoom: 19,
    });

    // Add Satellite tile layer
    const satellite = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
      attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
      maxZoom: 19,
    });

    const baseMaps = {
      "Карта": osm,
      "Спутник": satellite
    };

    osm.addTo(map);
    L.control.layers(baseMaps, {}, { position: 'topright' }).addTo(map);

    mapRef.current = map;

    // Create custom drone icon
    const droneIcon = L.icon({
      iconUrl:
        "data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMzIiIGhlaWdodD0iMzIiIHZpZXdCb3g9IjAgMCAzMiAzMiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPGNpcmNsZSBjeD0iMTYiIGN5PSIxNiIgcj0iMTQiIGZpbGw9IiMwMDdhZjgiIHN0cm9rZT0id2hpdGUiIHN0cm9rZS13aWR0aD0iMiIvPgo8cGF0aCBkPSJNMTYgOEwxOSAxNEgxM0wxNiA4WiIgZmlsbD0id2hpdGUiLz4KPHBhdGggZD0iTTI0IDE2TDE4IDE5VjEzTDI0IDE2WiIgZmlsbD0id2hpdGUiLz4KPHBhdGggZD0iTTggMTZMMTQgMTlWMTNMOCAxNloiIGZpbGw9IndoaXRlIi8+CjxwYXRoIGQ9Ik0xNiAyNEwxMyAyMFYyNkwxNiAyNFoiIGZpbGw9IndoaXRlIi8+Cjwvc3ZnPg==",
      iconSize: [32, 32],
      iconAnchor: [16, 16],
      popupAnchor: [0, -16],
    });

    // Add drone marker
    const droneMarker = L.marker([dronePosition.lat, dronePosition.lng], {
      icon: droneIcon,
    })
      .addTo(map)
      .bindPopup("<b>📍 Позиция дрона</b>");

    droneMarkerRef.current = droneMarker;

    // Add path if exists
    if (path.length > 1) {
      const pathCoords = path.map((p) => [p.lat, p.lng] as [number, number]);
      const polyline = L.polyline(pathCoords, {
        color: "#007af8",
        weight: 3,
        opacity: 0.7,
        dashArray: "5, 5",
      }).addTo(map);

      pathPolylineRef.current = polyline;

      // Fit map to path
      const group = new L.FeatureGroup([droneMarker, polyline]);
      map.fitBounds(group.getBounds(), { padding: [50, 50] });
    }

    // Add click handler if onMapClick is provided
    if (onMapClick) {
      map.on("click", (e: L.LeafletMouseEvent) => {
        onMapClick(e.latlng.lat, e.latlng.lng);
      });
    }

    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
      }
    };
  }, []);

  // Handle onMapClick changes
  useEffect(() => {
    if (!mapRef.current) return;

    // Remove existing click handler
    mapRef.current.off("click");

    // Add new click handler if provided
    if (onMapClick) {
      mapRef.current.on("click", (e: L.LeafletMouseEvent) => {
        onMapClick(e.latlng.lat, e.latlng.lng);
      });
    }
  }, [onMapClick]);

  // Update drone position
  useEffect(() => {
    if (droneMarkerRef.current && mapRef.current) {
      droneMarkerRef.current.setLatLng([dronePosition.lat, dronePosition.lng]);
      // Optionally follow drone
      if (followDrone) {
        mapRef.current.panTo([dronePosition.lat, dronePosition.lng]);
      }
    }
  }, [dronePosition, followDrone]);

  // Update path
  useEffect(() => {
    if (!mapRef.current) return;

    // Remove old markers
    pathMarkersRef.current.forEach((marker) => marker.remove());
    pathMarkersRef.current = [];

    // Add new markers if showMarkers is true
    if (showMarkers) {
      path.forEach((p, idx) => {
        const marker = L.marker([p.lat, p.lng], {
          icon: L.divIcon({
            className: "custom-path-marker",
            html: `<div class="w-6 h-6 rounded-full border-2 border-white bg-primary shadow-lg flex items-center justify-center text-white text-[10px] font-bold">${idx + 1}</div>`,
            iconSize: [24, 24],
            iconAnchor: [12, 12],
          }),
        }).addTo(mapRef.current!);
        pathMarkersRef.current.push(marker);
      });
    }

    if (pathPolylineRef.current) {
      const pathCoords = path.map((p) => [p.lat, p.lng] as [number, number]);
      pathPolylineRef.current.setLatLngs(pathCoords);
    } else if (path.length > 1 && mapRef.current) {
      const pathCoords = path.map((p) => [p.lat, p.lng] as [number, number]);
      const polyline = L.polyline(pathCoords, {
        color: "#007af8",
        weight: 3,
        opacity: 0.7,
        dashArray: "5, 5",
      }).addTo(mapRef.current);

      pathPolylineRef.current = polyline;
    }
  }, [path, showMarkers]);

  // Update selected point marker
  useEffect(() => {
    if (!mapRef.current) return;

    if (selectedPoint) {
      if (selectedPointMarkerRef.current) {
        selectedPointMarkerRef.current.setLatLng([
          selectedPoint.lat,
          selectedPoint.lng,
        ]);
      } else {
        const marker = L.marker([selectedPoint.lat, selectedPoint.lng], {
          icon: L.divIcon({
            className: "custom-selected-marker",
            html: '<div class="w-8 h-8 rounded-full border-2 border-amber-400 bg-amber-300 shadow-lg animate-pulse"></div>',
            iconSize: [32, 32],
            iconAnchor: [16, 16],
          }),
        }).addTo(mapRef.current);
        selectedPointMarkerRef.current = marker;
      }
    } else {
      if (selectedPointMarkerRef.current) {
        mapRef.current.removeLayer(selectedPointMarkerRef.current);
        selectedPointMarkerRef.current = null;
      }
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
