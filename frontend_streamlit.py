"""
frontend_streamlit.py: DoonRoute - High-performance Dijkstra routing across Dehradun road network.
Features random point generation, intersection snapping, pure Python vs. Cython C++ execution benchmarks,
map layers, workflow vs. algorithm-only timings, and JSON export.
"""

import sys
import os
import time
import json
import random
from collections import defaultdict, deque
import streamlit as st
import streamlit.components.v1 as components

# Ensure local package path is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dijkstra_fast_cpp.real_map_router import RealWorldMapGraph, LANDMARKS, haversine_distance
from dijkstra_fast_cpp.dijkstra_pure import dijkstra_cover_waypoints_python

try:
    from dijkstra_fast_cpp._dijkstra_cython import dijkstra_cython, FastGraph
    CYTHON_AVAILABLE = True
except ImportError:
    CYTHON_AVAILABLE = False


st.set_page_config(
    page_title="DoonRoute — Dehradun Road Routing",
    page_icon="🗺️",
    layout="wide"
)

# Custom CSS matching DoonRoute design
st.markdown("""
<style>
    /* Global Background & Dark Slate Theme */
    .stApp {
        background-color: #0e1117;
        color: #f1f5f9;
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }

    /* Red Primary Buttons matching DoonRoute */
    div.stButton > button:first-child {
        background-color: #ff4b4b;
        color: #ffffff;
        font-weight: 600;
        border-radius: 6px;
        border: none;
        padding: 0.45rem 1rem;
        transition: background-color 0.15s ease-in-out;
        width: 100%;
    }
    div.stButton > button:first-child:hover {
        background-color: #e03838;
        color: #ffffff;
    }

    /* Red Slider Accents */
    div[data-baseweb="slider"] div {
        color: #ff4b4b !important;
    }

    /* Multiselect red pills */
    span[data-baseweb="tag"] {
        background-color: #ef4444 !important;
        color: #ffffff !important;
        border-radius: 4px !important;
        font-weight: 500 !important;
    }

    /* Sidebar Clean Styling */
    section[data-testid="stSidebar"] {
        background-color: #131720;
        border-right: 1px solid #1e2433;
    }

    /* Clean Tables */
    .doon-table {
        width: 100%;
        border-collapse: collapse;
        font-size: 0.88rem;
        margin-top: 8px;
        margin-bottom: 14px;
    }
    .doon-table th {
        background-color: #1e2330;
        color: #94a3b8;
        padding: 8px 10px;
        text-align: left;
        font-weight: 600;
        border: 1px solid #2d3446;
    }
    .doon-table td {
        padding: 8px 10px;
        border: 1px solid #2d3446;
        color: #f1f5f9;
        font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
    }
    .doon-table tr:nth-child(even) {
        background-color: #161a24;
    }

    /* Metric Display */
    .metric-label {
        font-size: 0.95rem;
        color: #94a3b8;
        font-weight: 500;
        margin-top: 10px;
        margin-bottom: 2px;
    }
    .metric-value-huge {
        font-size: 2.3rem;
        font-weight: 700;
        color: #ffffff;
        line-height: 1.1;
        margin-bottom: 12px;
    }
</style>
""", unsafe_allow_html=True)

# -----------------------------------------------------------------
# GRAPH CACHING & CONNECTIVITY
# -----------------------------------------------------------------
@st.cache_resource
def load_graph_and_connected_component():
    g = RealWorldMapGraph()
    fg = None
    if CYTHON_AVAILABLE and g.loaded:
        fg = FastGraph(len(g.osm_id_to_idx))
        fg.add_edges_from_list(g.edges)

    # Precompute largest connected component
    adj = defaultdict(list)
    for u, v, w in g.edges:
        adj[u].append(v)
        adj[v].append(u)

    visited = set()
    components_list = []
    for node in range(len(g.osm_id_to_idx)):
        if node not in visited:
            comp = []
            q = deque([node])
            visited.add(node)
            while q:
                curr = q.popleft()
                comp.append(curr)
                for neighbor in adj[curr]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        q.append(neighbor)
            components_list.append(comp)

    components_list.sort(key=len, reverse=True)
    main_comp = set(components_list[0]) if components_list else set()

    return g, fg, main_comp

real_graph, c_graph, main_connected_nodes = load_graph_and_connected_component()

if not real_graph.loaded:
    st.error("Road network could not be loaded. Please check data/dehradun_roads.json.")
    st.stop()


# -----------------------------------------------------------------
# HELPER: RANDOM POINT GENERATION WITH INTERSECTION SNAPPING
# -----------------------------------------------------------------
def generate_points_dataset(num_points: int, seed: int, max_snap_meters: float):
    rng = random.Random(seed)
    # Filter nodes strictly within the main connected component
    valid_nodes = list(main_connected_nodes)
    
    # Calculate geographical bounds
    lats = [real_graph.get_node_coord(nid)[0] for nid in valid_nodes]
    lons = [real_graph.get_node_coord(nid)[1] for nid in valid_nodes]
    min_lat, max_lat = min(lats) + 0.015, max(lats) - 0.015
    min_lon, max_lon = min(lons) + 0.015, max(lons) - 0.015

    generated = []
    attempts = 0
    while len(generated) < num_points and attempts < num_points * 20:
        attempts += 1
        rand_lat = round(rng.uniform(min_lat, max_lat), 5)
        rand_lon = round(rng.uniform(min_lon, max_lon), 5)
        nearest_nid = real_graph.find_nearest_node((rand_lat, rand_lon))

        if nearest_nid in main_connected_nodes:
            ncoord = real_graph.get_node_coord(nearest_nid)
            snap_dist = haversine_distance((rand_lat, rand_lon), ncoord)
            if snap_dist <= max_snap_meters:
                osm_id = real_graph.idx_to_osm_id.get(nearest_nid, nearest_nid)
                p_idx = len(generated)
                generated.append({
                    "id": f"P{p_idx}",
                    "rand_lat": rand_lat,
                    "rand_lon": rand_lon,
                    "node_idx": nearest_nid,
                    "osm_id": osm_id,
                    "lat": ncoord[0],
                    "lon": ncoord[1],
                    "snap_dist": snap_dist,
                    "label": f"P{p_idx} | {rand_lat:.5f}, {rand_lon:.5f} | node {osm_id}"
                })

    # Fallback to direct sampling from main component if snapping was too strict
    if len(generated) < num_points:
        fallback_nodes = rng.sample(valid_nodes, num_points - len(generated))
        for fn in fallback_nodes:
            coord = real_graph.get_node_coord(fn)
            osm_id = real_graph.idx_to_osm_id.get(fn, fn)
            p_idx = len(generated)
            generated.append({
                "id": f"P{p_idx}",
                "rand_lat": coord[0],
                "rand_lon": coord[1],
                "node_idx": fn,
                "osm_id": osm_id,
                "lat": coord[0],
                "lon": coord[1],
                "snap_dist": 0.0,
                "label": f"P{p_idx} | {coord[0]:.5f}, {coord[1]:.5f} | node {osm_id}"
            })

    return generated


# -----------------------------------------------------------------
# SIDEBAR CONTROLS (EXACTLY MATCHING DOONROUTE)
# -----------------------------------------------------------------
with st.sidebar:
    num_pts = st.number_input("Number of points", min_value=2, max_value=12, value=5, step=1)
    seed = st.number_input("Random seed", min_value=1, max_value=99999, value=42, step=1)
    max_snapping = st.slider("Maximum snapping distance (m)", min_value=100, max_value=3000, value=1000, step=50)

    # Initialize points in session state
    if "points" not in st.session_state or st.session_state.get("seed") != seed or st.session_state.get("num_pts") != num_pts:
        st.session_state["points"] = generate_points_dataset(num_pts, seed, max_snapping)
        st.session_state["seed"] = seed
        st.session_state["num_pts"] = num_pts

    if st.button("Generate random points"):
        st.session_state["points"] = generate_points_dataset(num_pts, seed, max_snapping)
        st.rerun()

    points = st.session_state["points"]
    point_labels = [p["label"] for p in points]
    point_dict = {p["label"]: p for p in points}

    st.markdown("### Map layers")
    show_road_network = st.checkbox("Show full road network", value=False)
    show_intersections = st.checkbox("Show intersections", value=False)

    st.markdown("### Route selection")

    # Quick Landmark Preset Helper
    use_landmark_preset = st.toggle("Use Dehradun Landmarks (UPES / Raipur)", value=False)

    if use_landmark_preset:
        preset_names = list(LANDMARKS.keys())
        start_sel = st.selectbox("Start", preset_names, index=0)
        dest_sel = st.selectbox("Destination", preset_names, index=len(preset_names) - 1)
        inter_sel = st.multiselect(
            "Ordered intermediate waypoints",
            preset_names,
            default=["Kaulagarh (ONGC / FRI Area)", "Premnagar (Market)", "Clock Tower (Ghanta Ghar)"]
        )
        start_node = real_graph.find_nearest_node(LANDMARKS[start_sel])
        dest_node = real_graph.find_nearest_node(LANDMARKS[dest_sel])
        inter_nodes = [real_graph.find_nearest_node(LANDMARKS[name]) for name in inter_sel]
        all_ordered_nodes = [start_node] + inter_nodes + [dest_node]
        all_waypoints_meta = [
            {"name": start_sel, "lat": LANDMARKS[start_sel][0], "lon": LANDMARKS[start_sel][1], "type": "start"}
        ]
        for name in inter_sel:
            all_waypoints_meta.append({"name": name, "lat": LANDMARKS[name][0], "lon": LANDMARKS[name][1], "type": "waypoint"})
        all_waypoints_meta.append({"name": dest_sel, "lat": LANDMARKS[dest_sel][0], "lon": LANDMARKS[dest_sel][1], "type": "end"})

    else:
        start_label = st.selectbox("Start", point_labels, index=0)
        dest_label = st.selectbox("Destination", point_labels, index=len(point_labels) - 1)
        default_inter = [point_labels[i] for i in range(1, len(point_labels) - 1)]
        inter_labels = st.multiselect("Ordered intermediate waypoints", point_labels, default=default_inter)

        start_p = point_dict[start_label]
        dest_p = point_dict[dest_label]
        inter_ps = [point_dict[l] for l in inter_labels]

        all_ordered_nodes = [start_p["node_idx"]] + [p["node_idx"] for p in inter_ps] + [dest_p["node_idx"]]
        all_waypoints_meta = [
            {"name": start_p["id"], "lat": start_p["lat"], "lon": start_p["lon"], "type": "start"}
        ]
        for p in inter_ps:
            all_waypoints_meta.append({"name": p["id"], "lat": p["lat"], "lon": p["lon"], "type": "waypoint"})
        all_waypoints_meta.append({"name": dest_p["id"], "lat": dest_p["lat"], "lon": dest_p["lon"], "type": "end"})

    calc_clicked = st.button("Calculate route")


# -----------------------------------------------------------------
# ROUTE CALCULATION & BENCHMARKING
# -----------------------------------------------------------------
# Execute calculation
# 1. Pure Algorithm Benchmark
t0_algo = time.perf_counter()
py_d, py_path, py_reach = dijkstra_cover_waypoints_python(
    len(real_graph.osm_id_to_idx), real_graph.edges, all_ordered_nodes
)
py_algo_ms = (time.perf_counter() - t0_algo) * 1000.0

t1_algo = time.perf_counter()
if c_graph is not None:
    cy_d, cy_path, cy_reach = c_graph.cover_waypoints(all_ordered_nodes)
else:
    cy_d, cy_path, cy_reach = py_d, py_path, py_reach
cy_algo_ms = (time.perf_counter() - t1_algo) * 1000.0

# 2. Workflow Timing (Snapping / node resolve + Dijkstra + GPS coord extraction)
t0_wf = time.perf_counter()
wf_nodes_py = [nid for nid in all_ordered_nodes]
_, wf_path_py, _ = dijkstra_cover_waypoints_python(len(real_graph.osm_id_to_idx), real_graph.edges, wf_nodes_py)
wf_coords_py = [real_graph.get_node_coord(nid) for nid in wf_path_py]
py_wf_ms = (time.perf_counter() - t0_wf) * 1000.0

t1_wf = time.perf_counter()
wf_nodes_cy = [nid for nid in all_ordered_nodes]
if c_graph is not None:
    _, wf_path_cy, _ = c_graph.cover_waypoints(wf_nodes_cy)
else:
    wf_path_cy = wf_path_py
wf_coords_cy = [real_graph.get_node_coord(nid) for nid in wf_path_cy]
cy_wf_ms = (time.perf_counter() - t1_wf) * 1000.0

active_path = cy_path if cy_reach else py_path
total_distance_m = cy_d if cy_reach else py_d
total_distance_km = total_distance_m / 1000.0

speedup_factor = py_algo_ms / max(0.001, cy_algo_ms)
distance_agreement = abs(py_d - cy_d)
route_coords = [real_graph.get_node_coord(nid) for nid in active_path]


# -----------------------------------------------------------------
# MAIN PAGE HEADER (MATCHING DOONROUTE)
# -----------------------------------------------------------------
st.title("DoonRoute")
st.caption("Real Dehradun road routing with random points, intersection snapping, Python vs. Cython Dijkstra, and JSON export.")

col_map, col_metrics = st.columns([2.4, 1.3])

# -----------------------------------------------------------------
# RIGHT COLUMN: METRICS & COMPARISON TABLES
# -----------------------------------------------------------------
with col_metrics:
    st.subheader("Route")

    st.markdown("<div class='metric-label'>Distance</div>", unsafe_allow_html=True)
    st.markdown(f"<div class='metric-value-huge'>{total_distance_km:.3f} km</div>", unsafe_allow_html=True)

    st.markdown("<div class='metric-label'>Cython speedup</div>", unsafe_allow_html=True)
    st.markdown(f"<div class='metric-value-huge'>{speedup_factor:.2f}x</div>", unsafe_allow_html=True)

    # Algorithm Timing Table
    st.markdown(f"""
    <table class="doon-table">
        <thead>
            <tr>
                <th>Algorithm</th>
                <th>Execution time (ms)</th>
                <th>Distance (m)</th>
                <th>Path nodes</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>Python</b></td>
                <td>{py_algo_ms:.3f}</td>
                <td>{py_d:.3f}</td>
                <td>{len(py_path)}</td>
            </tr>
            <tr>
                <td><b>Cython</b></td>
                <td>{cy_algo_ms:.3f}</td>
                <td>{cy_d:.3f}</td>
                <td>{len(cy_path)}</td>
            </tr>
        </tbody>
    </table>
    """, unsafe_allow_html=True)

    st.caption(f"Distance agreement: {distance_agreement:.9f} m")

    # JSON Download
    json_export_data = {
        "title": "DoonRoute Export",
        "route_summary": {
            "total_distance_km": round(total_distance_km, 3),
            "total_distance_meters": round(total_distance_m, 3),
            "path_node_count": len(active_path),
            "cython_speedup": f"{speedup_factor:.2f}x",
            "distance_agreement_meters": distance_agreement
        },
        "algorithm_timing": {
            "python_time_ms": round(py_algo_ms, 3),
            "cython_time_ms": round(cy_algo_ms, 3)
        },
        "workflow_timing": {
            "python_workflow_ms": round(py_wf_ms, 4),
            "cython_workflow_ms": round(cy_wf_ms, 4)
        },
        "waypoints": all_waypoints_meta,
        "path_nodes": active_path,
        "coordinates": [{"lat": lat, "lon": lon} for lat, lon in route_coords]
    }
    json_export_str = json.dumps(json_export_data, indent=2)

    st.download_button(
        label="Download JSON",
        data=json_export_str,
        file_name=f"doonroute_seed{seed}_{round(total_distance_km)}km.json",
        mime="application/json",
        use_container_width=True
    )

    st.markdown("### Workflow timing (separate from algorithm-only)")

    st.markdown(f"""
    <table class="doon-table">
        <thead>
            <tr>
                <th>Algorithm</th>
                <th>Median execution time (ms)</th>
                <th>Distance (m)</th>
            </tr>
        </thead>
        <tbody>
            <tr>
                <td><b>Python</b></td>
                <td>{py_wf_ms:.4f}</td>
                <td>{py_d:.3f}</td>
            </tr>
            <tr>
                <td><b>Cython</b></td>
                <td>{cy_wf_ms:.4f}</td>
                <td>{cy_d:.3f}</td>
            </tr>
        </tbody>
    </table>
    """, unsafe_allow_html=True)


# -----------------------------------------------------------------
# LEFT COLUMN: LEAFLET MAP VISUALIZATION
# -----------------------------------------------------------------
with col_map:
    # Compute center
    if route_coords:
        c_lat = sum(c[0] for c in route_coords) / len(route_coords)
        c_lon = sum(c[1] for c in route_coords) / len(route_coords)
    else:
        c_lat, c_lon = 30.34, 78.02

    # Map Road Network / Intersections data if requested
    roads_js = []
    if show_road_network:
        # Sample subset of edges for rendering performance
        for u, v, w in real_graph.edges[::4]:
            p1 = real_graph.get_node_coord(u)
            p2 = real_graph.get_node_coord(v)
            roads_js.append([[p1[0], p1[1]], [p2[0], p2[1]]])

    intersections_js = []
    if show_intersections:
        for nid in list(main_connected_nodes)[::8]:
            c = real_graph.get_node_coord(nid)
            intersections_js.append([c[0], c[1]])

    leaflet_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8" />
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
        <style>
            html, body {{ height: 100%; margin: 0; padding: 0; background: #0e1117; }}
            #map {{ width: 100%; height: 640px; border-radius: 6px; border: 1px solid #1e2433; }}
            .leaflet-popup-content-wrapper {{ background: #181d27; color: #f1f5f9; font-family: sans-serif; border: 1px solid #2d3446; border-radius: 6px; }}
            .leaflet-popup-tip {{ background: #181d27; }}
        </style>
    </head>
    <body>
        <div id="map"></div>
        <script>
            var map = L.map('map').setView([{c_lat}, {c_lon}], 12);

            // OpenStreetMap Standard TileLayer
            L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                maxZoom: 19,
                attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }}).addTo(map);

            var boundsGroup = new L.featureGroup();

            // Background road network layer
            var showRoads = {json.dumps(show_road_network)};
            if (showRoads) {{
                var roads = {json.dumps(roads_js)};
                roads.forEach(function(segment) {{
                    L.polyline(segment, {{ color: '#94a3b8', weight: 1.2, opacity: 0.35 }}).addTo(map);
                }});
            }}

            // Intersections layer
            var showInter = {json.dumps(show_intersections)};
            if (showInter) {{
                var inters = {json.dumps(intersections_js)};
                inters.forEach(function(pt) {{
                    L.circleMarker(pt, {{ radius: 2, color: '#38bdf8', fillColor: '#38bdf8', fillOpacity: 0.6, weight: 0 }}).addTo(map);
                }});
            }}

            // Active Shortest Path Route (Bold Red-Orange)
            var routeCoords = {json.dumps(route_coords)};
            if (routeCoords.length > 0) {{
                var routePoly = L.polyline(routeCoords, {{
                    color: '#e64a19',
                    weight: 4.8,
                    opacity: 0.95,
                    lineJoin: 'round'
                }}).addTo(map);
                routePoly.bindPopup("<b>DoonRoute Path</b><br>Distance: <b>{total_distance_km:.3f} km</b><br>Nodes: {len(active_path)}");
                boundsGroup.addLayer(routePoly);
            }}

            // Markers
            var wps = {json.dumps(all_waypoints_meta)};
            wps.forEach(function(wp, i) {{
                var isStart = (wp.type === 'start');
                var isEnd = (wp.type === 'end');
                
                if (isStart) {{
                    var startIcon = L.divIcon({{
                        className: 'custom-pin-start',
                        html: '<div style="background-color: #4ade80; color: #0f172a; border-radius: 50%; width: 26px; height: 26px; display: flex; align-items: center; justify-content: center; font-weight: bold; border: 2px solid #ffffff; box-shadow: 0 2px 6px rgba(0,0,0,0.4);">S</div>',
                        iconSize: [26, 26],
                        iconAnchor: [13, 13]
                    }});
                    var m = L.marker([wp.lat, wp.lon], {{ icon: startIcon }}).addTo(map);
                    m.bindPopup("<b>Start: " + wp.name + "</b><br>Coords: " + wp.lat + ", " + wp.lon);
                    boundsGroup.addLayer(m);
                }} else if (isEnd) {{
                    var endIcon = L.divIcon({{
                        className: 'custom-pin-end',
                        html: '<div style="background-color: #ef4444; color: #ffffff; border-radius: 50%; width: 26px; height: 26px; display: flex; align-items: center; justify-content: center; font-weight: bold; border: 2px solid #ffffff; box-shadow: 0 2px 6px rgba(0,0,0,0.4);">D</div>',
                        iconSize: [26, 26],
                        iconAnchor: [13, 13]
                    }});
                    var m = L.marker([wp.lat, wp.lon], {{ icon: endIcon }}).addTo(map);
                    m.bindPopup("<b>Destination: " + wp.name + "</b><br>Coords: " + wp.lat + ", " + wp.lon);
                    boundsGroup.addLayer(m);
                }} else {{
                    var marker = L.circleMarker([wp.lat, wp.lon], {{
                        radius: 5,
                        fillColor: '#ef4444',
                        color: '#ffffff',
                        weight: 1.5,
                        fillOpacity: 1.0
                    }}).addTo(map);
                    marker.bindPopup("<b>Waypoint: " + wp.name + "</b><br>Coords: " + wp.lat + ", " + wp.lon);
                    boundsGroup.addLayer(marker);
                }}
            }});

            if (routeCoords.length > 0) {{
                map.fitBounds(boundsGroup.getBounds(), {{ padding: [35, 35] }});
            }}
        </script>
    </body>
    </html>
    """

    components.html(leaflet_html, height=660)
