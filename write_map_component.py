import base64
import sys

content = """import { useEffect, useRef } from 'react';
import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';

interface MapComponentProps {
  dronePosition?: { lat: number, lng: number };
  path?: Array<{ lat: number, lng: number }>;
  onMapClick?: (lat: number, lng: number) => void;
  selectedPoint?: { lat: number, lng: number };
  showMarkers?: boolean;
  followDrone?: boolean;
}

export function MapComponent({
  dronePosition = { lat: 55.7558, lng: 37.6173 },
  path = [],
  onMapClick,
  selectedPoint,
  showMarkers = true,
  followDrone = true,
}: MapComponentProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<maplibregl.Map | null>(null);
  const markerRef = useRef<maplibregl.Marker | null>(null);
  const selectedMarkerRef = useRef<maplibregl.Marker | null>(null);
  const didInit = useRef(false);

  const tileUrl = window.location.origin + '/api/map/tiles/{z}/{x}/{y}.pbf';

  useEffect(() => {
    if (!containerRef.current || mapRef.current || didInit.current) return;
    didInit.current = true;

    const map = new maplibregl.Map({
      container: containerRef.current,
      style: {
        version: 8,
        name: 'Offline Map',
        sources: {
          'local-tiles': {
            type: 'vector',
            tiles: [tileUrl],
            maxzoom: 14,
          },
        },
        layers: [
          { id: 'background', type: 'background', paint: { 'background-color': '#f2efe9' } },
          { id: 'water', type: 'fill', source: 'local-tiles', 'source-layer': 'water', paint: { 'fill-color': '#aadaff' } },
          { id: 'waterway', type: 'line', source: 'local-tiles', 'source-layer': 'waterway', paint: { 'line-color': '#aadaff', 'line-width': { base: 1.2, stops: [[8, 0.8], [14, 2]] } } },
          { id: 'landcover', type: 'fill', source: 'local-tiles', 'source-layer': 'landcover', paint: { 'fill-color': '#eef5e6' } },
          { id: 'landuse', type: 'fill', source: 'local-tiles', 'source-layer': 'landuse', paint: { 'fill-color': '#dde8c4' } },
          { id: 'park', type: 'fill', source: 'local-tiles', 'source-layer': 'park', paint: { 'fill-color': '#cde2a8' } },
          { id: 'aeroway', type: 'fill', source: 'local-tiles', 'source-layer': 'aeroway', filter: ['==', 'class', 'aerodrome'], paint: { 'fill-color': '#d9d0c9', 'fill-opacity': 0.6 } },
          { id: 'building', type: 'fill', source: 'local-tiles', 'source-layer': 'building', paint: { 'fill-color': '#d9d0c9', 'fill-opacity': 0.9 } },
          { id: 'road', type: 'line', source: 'local-tiles', 'source-layer': 'transportation', filter: ['has', 'class'], paint: { 'line-color': '#ffffff', 'line-width': { base: 1.5, stops: [[5, 1], [14, 8]] }, 'line-opacity': 0.9 } },
          { id: 'road-case', type: 'line', source: 'local-tiles', 'source-layer': 'transportation', filter: ['has', 'class'], paint: { 'line-color': '#b9b0a8', 'line-width': { base: 1.5, stops: [[5, 1.2], [14, 9]] }, 'line-opacity': 0.6 } },
          { id: 'boundary', type: 'line', source: 'local-tiles', 'source-layer': 'boundary', paint: { 'line-color': '#f85149', 'line-width': { base: 1.2, stops: [[6, 1], [12, 2.5]] }, 'line-dasharray': [3, 2] } },
          { id: 'poi', type: 'circle', source: 'local-tiles', 'source-layer': 'poi', paint: { 'circle-radius': { base: 1.3, stops: [[10, 2], [14, 5]] }, 'circle-color': '#bc8cff', 'circle-stroke-width': 1, 'circle-stroke-color': '#ffffff' } },
        ],
      },
      center: [dronePosition.lng, dronePosition.lat],
      zoom: 17,
    });

    const el = document.createElement('div');
    el.innerHTML = '<div style="font-size:24px;line-height:1;filter:drop-shadow(0 0 4px #3fb950)">\\uD83E\\uDD96</div>';
    markerRef.current = new maplibregl.Marker({ element: el })
      .setLngLat([dronePosition.lng, dronePosition.lat])
      .addTo(map);

    if (onMapClick) {
      map.on('click', (e) => {
        onMapClick(e.lngLat.lat, e.lngLat.lng);
      });
    }

    mapRef.current = map;

    return () => {
      map.remove();
      mapRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (markerRef.current && mapRef.current) {
      markerRef.current.setLngLat([dronePosition.lng, dronePosition.lat]);
      if (followDrone) {
        mapRef.current.panTo([dronePosition.lng, dronePosition.lat]);
      }
    }
  }, [dronePosition, followDrone]);

  useEffect(() => {
    if (selectedPoint) {
      if (!selectedMarkerRef.current) {
        const el = document.createElement('div');
        el.innerHTML = '<div style="font-size:20px;filter:drop-shadow(0 0 3px #f85149)">\\uD83D\\uDCCC</div>';
        selectedMarkerRef.current = new maplibregl.Marker({ element: el })
          .setLngLat([selectedPoint.lng, selectedPoint.lat])
          .addTo(mapRef.current!);
      } else {
        selectedMarkerRef.current.setLngLat([selectedPoint.lng, selectedPoint.lat]);
      }
    } else if (selectedMarkerRef.current) {
      selectedMarkerRef.current.remove();
      selectedMarkerRef.current = null;
    }
  }, [selectedPoint]);

  useEffect(() => {
    if (!mapRef.current) return;

    const sourceId = 'drone-path';
    const layerId = 'drone-path-layer';

    const applyPath = () => {
      if (!mapRef.current!.getSource(sourceId)) {
        mapRef.current!.addSource(sourceId, {
          type: 'geojson',
          data: {
            type: 'Feature',
            geometry: { type: 'LineString', coordinates: [] },
          },
        });
        mapRef.current!.addLayer({
          id: layerId,
          type: 'line',
          source: sourceId,
          paint: {
            'line-color': '#007af8',
            'line-width': 3,
            'line-opacity': 0.7,
            'line-dasharray': [2, 2],
          },
        });
      }

      const source = mapRef.current!.getSource(sourceId) as any;
      if (source && path.length > 1) {
        source.setData({
          type: 'Feature',
          geometry: {
            type: 'LineString',
            coordinates: path.map((p) => [p.lng, p.lat]),
          },
        });
      }
    };

    if (mapRef.current.loaded()) {
      applyPath();
    } else {
      mapRef.current.once('load', applyPath);
    }
  }, [path]);

  return <div ref={containerRef} style={{ width: '100%', height: '100%', minHeight: '420px' }} />;
}
"""

with open('/opt/main/Trajectory/TWA/client/components/MapComponent.tsx', 'w') as f:
    f.write(content)
