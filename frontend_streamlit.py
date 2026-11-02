"""
frontend_streamlit.py: Interactive web interface for IteroPath (dijkstra-fast-cpp).
High-performance C++ Dijkstra navigation over the OpenStreetMap Greater Dehradun road network.
Features real-time Cython vs Python benchmarking, exact distance agreement, interactive maps,
turn-by-turn directions, and JSON route export.
"""

import sys
import os
import time
import json
import pandas as pd
import streamlit as st
import streamlit.components.v1 as components

# Ensure local package path is importable
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dijkstra_fast_cpp.real_map_router import RealWorldMapGraph, LANDMARKS, haversine_distance
from dijkstra_fast_cpp.dijkstra_pure import dijkstra_python, dijkstra_cover_waypoints_python

try:
    from dijkstra_fast_cpp._dijkstra_cython import dijkstra_cython, FastGraph
    CYTHON_AVAILABLE = True
except ImportError:
    CYTHON_AVAILABLE = False


st.set_page_config(
    page_title="IteroPath — C++ Dijkstra Navigation",
    page_icon="🧭",
    layout="wide"
)

# Refined Professional Dark Theme Styling
st.markdown("""
<style>
    /* Typography & Base */
    html, body, [class*="css"] {
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }

    /* Streamlit Main Container Spacing */
    .block-container {
        padding-top: 1.8rem;
        padding-bottom: 2rem;
    }

    /* Badges */
    .badge-agreement {
        background: #064e3b;
        color: #34d399;
        font-size: 0.85rem;
        font-weight: 600;
        padding: 5px 12px;
        border-radius: 6px;
        border: 1px solid #059669;
        display: inline-block;
        margin-top: 6px;
        margin-bottom: 10px;
    }
    .speedup-badge {
        color: #38bdf8;
        font-size: 1.5rem;
        font-weight: 700;
        letter-spacing: -0.02em;
    }

    /* Checkpoint Card */
    .checkpoint-card {
        background-color: #161922;
        border: 1px solid #232734;
        border-left: 3px solid #38bdf8;
        padding: 8px 12px;
        margin-bottom: 6px;
        border-radius: 6px;
    }
    .checkpoint-title {
        font-weight: 600;
        color: #f8fafc;
        font-size: 0.9rem;
    }
    .checkpoint-meta {
        color: #64748b;
        font-size: 0.8rem;
        font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
        margin-top: 2px;
    }
    .checkpoint-dist {
        color: #38bdf8;
        font-weight: 600;
        float: right;
        font-size: 0.88rem;
    }

    /* Expander Clean Border */
    div[data-testid="stExpander"] {
        border: 1px solid #232734;
        border-radius: 8px;
        background-color: #12151d;
        margin-bottom: 8px;
    }

    /* Tabs Styling */
    .stTabs [data-baseweb="tab-list"] {
        gap: 20px;
        border-bottom: 1px solid #232734;
    }
    .stTabs [data-baseweb="tab"] {
        height: 42px;
        font-weight: 600;
        font-size: 0.95rem;
    }
</style>
""", unsafe_allow_html=True)

# Top Header
st.title("IteroPath")
st.caption("High-Performance C++ Dijkstra Navigation & Corridor Engine (Cython vs Pure Python)")

# Cache graph loading
@st.cache_resource
def load_real_graph():
    g = RealWorldMapGraph()
    fg = None
    if CYTHON_AVAILABLE and g.loaded:
        fg = FastGraph(len(g.osm_id_to_idx))
        fg.add_edges_from_list(g.edges)
    return g, fg

real_graph, c_graph = load_real_graph()

if not real_graph.loaded:
    st.error("Road network data could not be loaded. Please verify data/dehradun_roads.json.")
    st.stop()

# =============================================================================
# SIDEBAR CONFIGURATION
# =============================================================================
st.sidebar.subheader("Route Configuration")
landmarks = list(LANDMARKS.keys())

trip_type = st.sidebar.radio(
    "Navigation Type",
    ["Direct Trip (Multi-Corridor)", "Multi-Stop Waypoint Tour"]
)

if trip_type == "Direct Trip (Multi-Corridor)":
    origin_name = st.sidebar.selectbox("Start Location", landmarks, index=0)
    dest_options = [lm for lm in landmarks if lm != origin_name]
    default_dest_idx = dest_options.index("Clock Tower (Ghanta Ghar)") if "Clock Tower (Ghanta Ghar)" in dest_options else 0
    dest_name = st.sidebar.selectbox("Destination", dest_options, index=default_dest_idx)

    num_routes_to_find = st.sidebar.slider(
        "Corridors to Explore",
        min_value=1,
        max_value=9,
        value=5,
        help="Discovers the optimal shortest path and diverse alternative corridors"
    )

    start_idx = real_graph.find_nearest_node(LANDMARKS[origin_name])
    end_idx = real_graph.find_nearest_node(LANDMARKS[dest_name])
    routes = real_graph.discover_all_routes(start_idx, end_idx, c_graph, max_alternatives=num_routes_to_find)

else:
    # Multi-Stop Tour
    st.sidebar.info("Select 3 to 6 ordered checkpoints across Greater Dehradun.")
    selected_stops = st.sidebar.multiselect(
        "Tour Stops (Ordered)",
        landmarks,
        default=[
            "UPES Bidholi (Campus)",
            "Kaulagarh (ONGC / FRI Area)",
            "Premnagar (Market)",
            "Clock Tower (Ghanta Ghar)",
            "Raipur Road (Ordnance / Raipur)"
        ]
    )

    if len(selected_stops) < 2:
        st.warning("Please select at least 2 checkpoints to generate a route.")
        st.stop()

    origin_name = selected_stops[0]
    dest_name = selected_stops[-1]

    tour_node_indices = [real_graph.find_nearest_node(LANDMARKS[s]) for s in selected_stops]

    t0 = time.perf_counter()
    py_dist, py_path, py_reach = dijkstra_cover_waypoints_python(
        len(real_graph.osm_id_to_idx), real_graph.edges, tour_node_indices
    )
    py_time_ms = (time.perf_counter() - t0) * 1000.0

    if c_graph:
        t1 = time.perf_counter()
        cy_dist, cy_path, cy_reach = c_graph.cover_waypoints(tour_node_indices)
        cy_time_ms = (time.perf_counter() - t1) * 1000.0
    else:
        cy_dist, cy_path, cy_reach = py_dist, py_path, py_reach
        cy_time_ms = py_time_ms

    speedup_val = py_time_ms / max(0.0001, cy_time_ms)
    coords = [real_graph.get_node_coord(nid) for nid in cy_path]
    directions = real_graph.generate_directions(cy_path)

    tour_route = {
        "id": "tour",
        "name": f"Tour: {' -> '.join([s.split()[0] for s in selected_stops])}",
        "distance_km": round(cy_dist / 1000.0, 3),
        "distance_m": cy_dist,
        "delta_km": 0.0,
        "is_optimal": True,
        "color": "#38bdf8",
        "dash_array": None,
        "description": f"Sequential traversal across {len(selected_stops)} landmarks",
        "path_indices": cy_path,
        "gps_coordinates": coords,
        "node_count": len(cy_path),
        "intermediate_landmarks": [
            {
                "name": s,
                "node_id": real_graph.find_nearest_node(LANDMARKS[s]),
                "lat": LANDMARKS[s][0],
                "lon": LANDMARKS[s][1],
                "distance_from_start_km": 0.0
            }
            for s in selected_stops
        ],
        "directions": directions,
        "cython_time_ms": cy_time_ms,
        "python_time_ms": py_time_ms,
        "speedup": f"{speedup_val:.1f}x"
    }
    routes = [tour_route]

if not routes:
    st.error("No route found between the selected locations.")
    st.stop()

# =============================================================================
# TABS: ROUTE EXPLORER & PERFORMANCE BENCHMARKS
# =============================================================================
tab_nav, tab_benchmarks = st.tabs([
    "Route Navigation & Verification",
    "Performance Benchmarks & Charts"
])

# -----------------------------------------------------------------------------
# TAB 1: ROUTE NAVIGATION & VERIFICATION
# -----------------------------------------------------------------------------
with tab_nav:
    col_map, col_details = st.columns([2.2, 1.3])

    with col_details:
        if len(routes) > 1:
            st.subheader("Route Selection")
            route_options = [
                f"{'[Optimal] ' if r['is_optimal'] else ''}{r['name']} — {r['distance_km']} km ({'+0.0 km' if r['is_optimal'] else '+' + str(r['delta_km']) + ' km'})"
                for r in routes
            ]
            selected_route_str = st.radio("Select Corridor", route_options, index=0, label_visibility="collapsed")
            selected_idx = route_options.index(selected_route_str)
            active_route = routes[selected_idx]
        else:
            active_route = routes[0]

        # Benchmark Timing & Parity Verification
        st.subheader("Performance & Verification")

        p_indices = active_route["path_indices"]
        start_n = p_indices[0]
        dest_n = p_indices[-1]

        if trip_type == "Multi-Stop Waypoint Tour":
            # Algorithm-only timing
            t0 = time.perf_counter()
            p_dist, p_p, p_reach = dijkstra_cover_waypoints_python(len(real_graph.osm_id_to_idx), real_graph.edges, tour_node_indices)
            live_py_algo_ms = (time.perf_counter() - t0) * 1000.0

            if c_graph:
                t1 = time.perf_counter()
                c_dist, c_p, c_reach = c_graph.cover_waypoints(tour_node_indices)
                live_cy_algo_ms = (time.perf_counter() - t1) * 1000.0
            else:
                c_dist, c_p = p_dist, p_p
                live_cy_algo_ms = live_py_algo_ms

            # Workflow timing
            t_wf0 = time.perf_counter()
            wf_nodes_py = [real_graph.find_nearest_node(LANDMARKS[s]) for s in selected_stops]
            _, wf_p_py, _ = dijkstra_cover_waypoints_python(len(real_graph.osm_id_to_idx), real_graph.edges, wf_nodes_py)
            _ = [real_graph.get_node_coord(nid) for nid in wf_p_py]
            py_wf_ms = (time.perf_counter() - t_wf0) * 1000.0

            t_wf1 = time.perf_counter()
            wf_nodes_cy = [real_graph.find_nearest_node(LANDMARKS[s]) for s in selected_stops]
            if c_graph:
                _, wf_p_cy, _ = c_graph.cover_waypoints(wf_nodes_cy)
            else:
                wf_p_cy = wf_p_py
            _ = [real_graph.get_node_coord(nid) for nid in wf_p_cy]
            cy_wf_ms = (time.perf_counter() - t_wf1) * 1000.0

        else:
            # Direct Trip timing
            t0 = time.perf_counter()
            p_dist, p_p, p_reach = dijkstra_python(len(real_graph.osm_id_to_idx), real_graph.edges, start_n, dest_n)
            live_py_algo_ms = (time.perf_counter() - t0) * 1000.0

            if c_graph:
                t1 = time.perf_counter()
                c_dist, c_p, c_reach = c_graph.shortest_path(start_n, dest_n)
                live_cy_algo_ms = (time.perf_counter() - t1) * 1000.0
            else:
                c_dist, c_p = p_dist, p_p
                live_cy_algo_ms = live_py_algo_ms

            # Workflow timing
            t_wf0 = time.perf_counter()
            s_snapped_py = real_graph.find_nearest_node(LANDMARKS[origin_name])
            e_snapped_py = real_graph.find_nearest_node(LANDMARKS[dest_name])
            _, wf_p_py, _ = dijkstra_python(len(real_graph.osm_id_to_idx), real_graph.edges, s_snapped_py, e_snapped_py)
            _ = [real_graph.get_node_coord(nid) for nid in wf_p_py]
            py_wf_ms = (time.perf_counter() - t_wf0) * 1000.0

            t_wf1 = time.perf_counter()
            s_snapped_cy = real_graph.find_nearest_node(LANDMARKS[origin_name])
            e_snapped_cy = real_graph.find_nearest_node(LANDMARKS[dest_name])
            if c_graph:
                _, wf_p_cy, _ = c_graph.shortest_path(s_snapped_cy, e_snapped_cy)
            else:
                wf_p_cy = wf_p_py
            _ = [real_graph.get_node_coord(nid) for nid in wf_p_cy]
            cy_wf_ms = (time.perf_counter() - t_wf1) * 1000.0

        live_speedup = live_py_algo_ms / max(0.0001, live_cy_algo_ms)
        live_delta = abs(p_dist - c_dist)

        # Top Speedup Stat
        st.markdown(
            f"Distance: **{active_route['distance_km']} km** &nbsp;|&nbsp; "
            f"Cython Speedup: <span class='speedup-badge'>{live_speedup:.2f}x</span>",
            unsafe_allow_html=True
        )

        perf_df = pd.DataFrame([
            {"Algorithm": "Pure Python", "Execution time (ms)": f"{live_py_algo_ms:.3f}", "Distance (m)": f"{p_dist:.3f}", "Path length": len(p_p)},
            {"Algorithm": "Cython C++", "Execution time (ms)": f"{live_cy_algo_ms:.3f}", "Distance (m)": f"{c_dist:.3f}", "Path length": len(c_p)}
        ])
        st.dataframe(perf_df, hide_index=True, width='stretch')

        st.markdown(f"<div class='badge-agreement'>Distance agreement: {live_delta:.9f} m</div>", unsafe_allow_html=True)

        with st.expander("Workflow Timing (Separate from Algorithm-Only)"):
            wf_df = pd.DataFrame([
                {"Algorithm": "Python Workflow", "Total Time (ms)": f"{py_wf_ms:.4f}"},
                {"Algorithm": "Cython Workflow", "Total Time (ms)": f"{cy_wf_ms:.4f}"}
            ])
            st.dataframe(wf_df, hide_index=True, width='stretch')

        # Turn-by-Turn Directions
        with st.expander("Turn-by-Turn Directions", expanded=False):
            for idx, s in enumerate(active_route["directions"]):
                st.write(f"**{idx + 1}.** {s['instruction']} — `{s['distance_km']} km` ({s['heading']})")

        # Checkpoints (if any)
        landmarks_passed = active_route["intermediate_landmarks"]
        if landmarks_passed:
            with st.expander(f"Intermediate Checkpoints ({len(landmarks_passed)})", expanded=False):
                for lm in landmarks_passed:
                    st.markdown(
                        f"<div class='checkpoint-card'>"
                        f"<span class='checkpoint-dist'>{lm['distance_from_start_km']} km</span>"
                        f"<div class='checkpoint-title'>{lm['name']}</div>"
                        f"<div class='checkpoint-meta'>Node #{lm['node_id']} · {lm['lat']:.4f}, {lm['lon']:.4f}</div>"
                        f"</div>",
                        unsafe_allow_html=True
                    )

        # JSON Export Button
        export_payload = {
            "origin": origin_name,
            "destination": dest_name,
            "distance_km": active_route["distance_km"],
            "node_count": active_route["node_count"],
            "speedup": f"{live_speedup:.2f}x",
            "distance_agreement_m": f"{live_delta:.9f}",
            "timing": {
                "python_algo_ms": live_py_algo_ms,
                "cython_algo_ms": live_cy_algo_ms,
                "python_workflow_ms": py_wf_ms,
                "cython_workflow_ms": cy_wf_ms
            },
            "coordinates": [{"lat": lat, "lon": lon} for lat, lon in active_route["gps_coordinates"]]
        }
        json_export_str = json.dumps(export_payload, indent=2)

        st.download_button(
            label="Download Route JSON",
            data=json_export_str,
            file_name=f"route_{origin_name.split()[0]}_to_{dest_name.split()[0]}.json",
            mime="application/json",
            width='stretch'
        )

    with col_map:
        st.subheader("OpenStreetMap Route View")
        st.caption("Cyan: Shortest Path · Dashed Lines: Alternative Corridors")

        start_coord = LANDMARKS[origin_name]
        end_coord = LANDMARKS[dest_name]
        center_lat = (start_coord[0] + end_coord[0]) / 2.0
        center_lon = (start_coord[1] + end_coord[1]) / 2.0

        routes_js_data = []
        for r in routes:
            routes_js_data.append({
                "id": r["id"],
                "name": r["name"],
                "distance_km": r["distance_km"],
                "is_optimal": r["is_optimal"],
                "color": r["color"],
                "dash": r["dash_array"],
                "weight": 5.5 if r["id"] == active_route["id"] else 3.2,
                "opacity": 0.95 if r["id"] == active_route["id"] else 0.55,
                "coords": [[lat, lon] for lat, lon in r["gps_coordinates"]]
            })

        pins_data = [{
            "name": f"Start: {origin_name}",
            "node_id": active_route["path_indices"][0],
            "lat": start_coord[0],
            "lon": start_coord[1],
            "km": 0.0,
            "type": "start"
        }]
        for lm in active_route["intermediate_landmarks"]:
            if lm["name"] != origin_name and lm["name"] != dest_name:
                pins_data.append({
                    "name": lm["name"],
                    "node_id": lm["node_id"],
                    "lat": lm["lat"],
                    "lon": lm["lon"],
                    "km": lm["distance_from_start_km"],
                    "type": "checkpoint"
                })
        pins_data.append({
            "name": f"Destination: {dest_name}",
            "node_id": active_route["path_indices"][-1],
            "lat": end_coord[0],
            "lon": end_coord[1],
            "km": active_route["distance_km"],
            "type": "end"
        })

        leaflet_html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8" />
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <style>
                html, body {{ height: 100%; margin: 0; padding: 0; background: #0f1117; }}
                #map {{ width: 100%; height: 620px; border-radius: 8px; border: 1px solid #232734; }}
                .leaflet-popup-content-wrapper {{ background: #161922; color: #f1f5f9; font-family: sans-serif; border: 1px solid #2d3345; border-radius: 6px; }}
                .leaflet-popup-tip {{ background: #161922; }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map = L.map('map').setView([{center_lat}, {center_lon}], 12);
                
                L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                    maxZoom: 19,
                    attribution: '© OpenStreetMap contributors'
                }}).addTo(map);

                var allRoutes = {json.dumps(routes_js_data)};
                var boundsGroup = new L.featureGroup();

                allRoutes.forEach(function(r) {{
                    var polyOptions = {{
                        color: r.color,
                        weight: r.weight,
                        opacity: r.opacity,
                        lineJoin: 'round'
                    }};
                    if (r.dash) {{
                        polyOptions.dashArray = r.dash;
                    }}
                    var poly = L.polyline(r.coords, polyOptions).addTo(map);
                    poly.bindPopup("<b>" + r.name + "</b><br>Distance: <b>" + r.distance_km + " km</b><br>" + (r.is_optimal ? "<span style='color:#34d399; font-weight:600;'>Optimal Shortest Path</span>" : "<span style='color:#94a3b8;'>Alternative Corridor</span>"));
                    boundsGroup.addLayer(poly);
                }});

                var pins = {json.dumps(pins_data)};
                pins.forEach(function(p) {{
                    var pinColor = p.type === 'start' ? '#10b981' : (p.type === 'end' ? '#ef4444' : '#f59e0b');
                    var marker = L.circleMarker([p.lat, p.lon], {{
                        radius: (p.type === 'start' || p.type === 'end') ? 8 : 5,
                        fillColor: pinColor,
                        color: '#ffffff',
                        weight: 2,
                        fillOpacity: 1
                    }}).addTo(map);
                    marker.bindPopup("<b>" + p.name + "</b><br>Node ID: <code>#" + p.node_id + "</code><br>Distance: <b>" + p.km + " km</b><br>Coordinates: " + p.lat + ", " + p.lon);
                    boundsGroup.addLayer(marker);
                }});

                map.fitBounds(boundsGroup.getBounds(), {{ padding: [35, 35] }});
            </script>
        </body>
        </html>
        """

        components.html(leaflet_html, height=640)


# -----------------------------------------------------------------------------
# TAB 2: PERFORMANCE BENCHMARKS & CHARTS
# -----------------------------------------------------------------------------
with tab_benchmarks:
    st.subheader("IteroPath Performance Benchmark Suite")
    st.caption("Empirical latency and speedup analysis on the real Dehradun road network (19,765 Nodes, 37,399 Edges).")

    results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    overall_png = os.path.join(results_dir, "overall_performance.png")
    route_png = os.path.join(results_dir, "route_comparison.png")
    exec_png = os.path.join(results_dir, "execution_time.png")
    speedup_png = os.path.join(results_dir, "speedup.png")
    synth_png = os.path.join(results_dir, "synthetic_scaling.png")
    route_csv = os.path.join(results_dir, "route_results.csv")

    if os.path.exists(overall_png):
        st.image(overall_png, caption="IteroPath Comprehensive Performance & Numerical Parity Dashboard", width='stretch')

    st.markdown("---")
    st.subheader("Real-World Dehradun Corridor Benchmarks")
    if os.path.exists(route_csv):
        df_routes = pd.read_csv(route_csv)
        st.dataframe(df_routes, hide_index=True, width='stretch')

    st.markdown("---")
    c1, c2 = st.columns(2)
    with c1:
        if os.path.exists(route_png):
            st.image(route_png, caption="Corridor Latency & Speedup on Dehradun Road Network", width='stretch')
    with c2:
        if os.path.exists(speedup_png):
            st.image(speedup_png, caption="Speedup Multipliers vs Graph Node Scale & Waypoints", width='stretch')

    st.markdown("---")
    c3, c4 = st.columns(2)
    with c3:
        if os.path.exists(exec_png):
            st.image(exec_png, caption="Dijkstra Execution Latency (Logarithmic Scale)", width='stretch')
    with c4:
        if os.path.exists(synth_png):
            st.image(synth_png, caption="Query Throughput Capacity & Algorithmic Complexity Alignment", width='stretch')
