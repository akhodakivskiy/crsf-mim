<script>
  let { target, interceptor } = $props();

  let map = $state();
  let targetMarker;
  let interceptorMarker;

  import { onMount } from "svelte";
  import L from "leaflet";
  import "leaflet/dist/leaflet.css";

  onMount(() => {
    // Initialize map centered on interceptor if available
    const center = interceptor
      ? [interceptor.lat, interceptor.lon]
      : [50.45, 30.52];

    map = L.map("map-view").setView(center, 13);

    // Add tiles
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: "&copy; OpenStreetMap contributors",
      maxZoom: 19,
    }).addTo(map);

    // Update markers when props change
    updateMarkers();
  });

  // Use $effect to track changes to target, interceptor, and map
  $effect(() => {
    if (map) {
      updateMarkers();
    }
  });

  function calculateHeading(vel_north, vel_east) {
    // Calculate heading in degrees (0° = North, 90° = East)
    const heading = Math.atan2(vel_east, vel_north) * (180 / Math.PI);
    return heading;
  }

  function calculateRelativeHeading(targetHeading, interceptorHeading) {
    let diff = targetHeading - interceptorHeading;
    // Normalize to -180 to +180 range
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
  }

  function formatWithSign(value) {
    return value >= 0 ? `+${value.toFixed(1)}` : value.toFixed(1);
  }

  function updateMarkers() {
    if (!map) return;

    // Update or create target marker
    if (target) {
      const targetHeading =
        target.velN !== undefined && target.velE !== undefined
          ? calculateHeading(target.velN, target.velE)
          : 0;

      let relativeInfo = "";
      if (interceptor) {
        const relativeAlt = (target.alt || 0) - (interceptor.alt || 0);
        const interceptorHeading = calculateHeading(
          interceptor.velN,
          interceptor.velE,
        );
        const relativeHeading = calculateRelativeHeading(
          targetHeading,
          interceptorHeading,
        );

        const altColor = relativeAlt >= 0 ? "#ff0000" : "#00cc00";
        const hdgColor = relativeHeading >= 0 ? "#ff9900" : "#00ccff";

        relativeInfo = `
          <div class="marker-label">
            <div style="color: ${altColor}">${formatWithSign(relativeAlt)}m</div>
            <div style="color: ${hdgColor}">${formatWithSign(relativeHeading)}°</div>
          </div>
        `;
      }

      const targetIcon = L.divIcon({
        html: `
          <div class="marker-container">
            <div class="arrow-icon" style="transform: rotate(${targetHeading}deg) scaleX(0.6); color: #ff0000;">▲</div>
            ${relativeInfo}
          </div>
        `,
        className: "target-marker",
        iconSize: [120, 24],
        iconAnchor: [12, 12],
      });

      const targetVel =
        target.velN !== undefined && target.velE !== undefined
          ? Math.sqrt(target.velN ** 2 + target.velE ** 2)
          : 0;

      const popupContent = `Target<br>Lat: ${target.lat.toFixed(6)}<br>Lon: ${target.lon.toFixed(6)}<br>Alt: ${(target.alt || 0).toFixed(1)}m<br>Vel: ${targetVel.toFixed(1)}m/s<br>Heading: ${targetHeading.toFixed(1)}°`;

      if (targetMarker) {
        // Update existing marker
        targetMarker.setLatLng([target.lat, target.lon]);
        targetMarker.setIcon(targetIcon);
        targetMarker.getPopup().setContent(popupContent);
      } else {
        // Create new marker
        targetMarker = L.marker([target.lat, target.lon], { icon: targetIcon })
          .addTo(map)
          .bindPopup(popupContent);
      }
    } else if (targetMarker) {
      // Remove marker if target is null
      map.removeLayer(targetMarker);
      targetMarker = null;
    }

    // Update or create interceptor marker
    if (interceptor) {
      const interceptorHeading = calculateHeading(
        interceptor.velN,
        interceptor.velE,
      );

      const interceptorIcon = L.divIcon({
        html: `<div class="arrow-icon" style="transform: rotate(${interceptorHeading}deg) scaleX(0.6); color: #0000ff;">▲</div>`,
        className: "interceptor-marker",
        iconSize: [24, 24],
        iconAnchor: [12, 12],
      });

      const popupContent = `Interceptor<br>Lat: ${interceptor.lat.toFixed(6)}<br>Lon: ${interceptor.lon.toFixed(6)}<br>Alt: ${(interceptor.alt || 0).toFixed(1)}m<br>Vel: ${Math.sqrt(interceptor.velN ** 2 + interceptor.velE ** 2).toFixed(1)}m/s<br>Heading: ${interceptorHeading.toFixed(1)}°`;

      if (interceptorMarker) {
        // Update existing marker
        interceptorMarker.setLatLng([interceptor.lat, interceptor.lon]);
        interceptorMarker.setIcon(interceptorIcon);
        interceptorMarker.getPopup().setContent(popupContent);
      } else {
        // Create new marker
        interceptorMarker = L.marker([interceptor.lat, interceptor.lon], {
          icon: interceptorIcon,
        })
          .addTo(map)
          .bindPopup(popupContent);
      }
    } else if (interceptorMarker) {
      // Remove marker if interceptor is null
      map.removeLayer(interceptorMarker);
      interceptorMarker = null;
    }
  }
</script>

<div id="map-view"></div>

<style>
  #map-view {
    width: 100%;
    height: 250px;
    position: relative;
    z-index: 1;
    border-radius: 4px;
    overflow: hidden;
  }

  :global(.target-marker) {
    color: #ff0000;
    font-size: 24px;
    text-align: center;
    line-height: 24px;
    filter: drop-shadow(0 0 2px rgba(0, 0, 0, 0.5));
  }

  :global(.target-marker .marker-container) {
    display: flex;
    align-items: center;
    gap: 8px;
  }

  :global(.target-marker .marker-label) {
    font-size: 10px;
    font-weight: bold;
    line-height: 1.2;
    white-space: nowrap;
    text-shadow:
      -1px -1px 2px rgba(0, 0, 0, 0.8),
      1px -1px 2px rgba(0, 0, 0, 0.8),
      -1px 1px 2px rgba(0, 0, 0, 0.8),
      1px 1px 2px rgba(0, 0, 0, 0.8);
  }

  :global(.interceptor-marker) {
    color: #0000ff;
    font-size: 24px;
    text-align: center;
    line-height: 24px;
    filter: drop-shadow(0 0 2px rgba(0, 0, 0, 0.5));
  }

  :global(.leaflet-popup-content-wrapper) {
    font-family: monospace;
    font-size: 11px;
  }

  :global(.leaflet-control-attribution) {
    font-size: 9px;
  }

  @media (min-width: 768px) {
    #map-view {
      height: 400px;
    }

    :global(.leaflet-popup-content-wrapper) {
      font-size: 12px;
    }
  }

  @media (min-width: 1024px) {
    #map-view {
      height: 500px;
    }
  }
</style>
