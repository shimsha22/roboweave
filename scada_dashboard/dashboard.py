import streamlit as st
import pandas as pd
import time

from core.cad_parser import DEFAULT_GRID, parse_cad_image
from core.bridge import init_ros_engine
from components.ui_charts import (
    render_floorplan_chart,
    render_telemetry_breakdown_chart,
    render_battery_chart,
    render_service_times_chart
)

st.set_page_config(page_title="NEXUS | SCADA Fleet Orchestrator", layout="wide", initial_sidebar_state="expanded")

st.markdown("""
<style>
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');
    html, body, [class*="css"], .stApp {
        font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
        background-color: #f8fafc;
        color: #1e293b;
    }
    .header-bar {
        background: #ffffff;
        border-bottom: 1px solid #e2e8f0;
        padding: 8px 16px;
        margin: -50px -50px 10px -50px;
        display: flex;
        align-items: center;
        justify-content: space-between;
    }
    .panel-title {
        font-size: 10px;
        font-weight: 700;
        text-transform: uppercase;
        letter-spacing: 0.6px;
        color: #64748b;
        margin-bottom: 4px;
        border-bottom: 1px solid #f1f5f9;
        padding-bottom: 2px;
    }
    .legend-item { display: inline-flex; align-items: center; font-size: 10px; font-weight: 500; color: #475569; margin-right: 8px; }
    .legend-box { width: 8px; height: 8px; border-radius: 2px; margin-right: 3px; }
</style>
""", unsafe_allow_html=True)

@st.cache_resource
def get_bridge():
    return init_ros_engine()

ros_node, ros_data = get_bridge()

if "grid" not in st.session_state:
    st.session_state.grid = DEFAULT_GRID
if "cad_name" not in st.session_state:
    st.session_state.cad_name = "DEFAULT_WAREHOUSE.dwg"

# --- SIDEBAR: WMS Mission Dispatch ---
st.sidebar.markdown("### Supervisor WMS Mission Console")

with ros_data["lock"]:
    available_robots = list(ros_data["fleet"].keys())

if available_robots:
    selected_robot = st.sidebar.selectbox("Select Target AMR", available_robots)
    
    active_grid = st.session_state.grid
    gh, gw = len(active_grid), len(active_grid[0])

    dynamic_stations = {
        f"INBOUND_DOCK (Row 0, Col 0)": [0, 0],
        f"OUTBOUND_BAY (Row {gh-1}, Col {gw-1})": [gh-1, gw-1],
        f"CENTRAL_CORRIDOR (Row {gh//2}, Col {gw//2})": [gh//2, gw//2],
        f"NORTH_FEEDER (Row 0, Col {gw-1})": [0, gw-1],
        f"SOUTH_BUFFER (Row {gh-1}, Col 0)": [gh-1, 0],
    }

    dispatch_mode = st.sidebar.radio("Destination Mode", ["Dynamic Bay Presets", "Custom Floor Coordinates"], horizontal=True)

    dest_coords = [0, 0]
    is_valid_point = True

    if dispatch_mode == "Dynamic Bay Presets":
        selected_stn = st.sidebar.selectbox("Destination Action Point", list(dynamic_stations.keys()))
        dest_coords = dynamic_stations[selected_stn]
    else:
        c1, c2 = st.sidebar.columns(2)
        target_r = c1.number_input("Target Row", 0, gh - 1, gh // 2)
        target_c = c2.number_input("Target Col", 0, gw - 1, gw // 2)
        dest_coords = [int(target_r), int(target_c)]
        
        if active_grid[dest_coords[0]][dest_coords[1]] == 1:
            st.sidebar.error("Selected coordinate is inside a structural obstacle!")
            is_valid_point = False

    sku_name = st.sidebar.text_input("Payload SKU / Cargo", "Medical Supplies (SKU-89)")
    prio = st.sidebar.slider("SLA Dispatch Priority", min_value=1, max_value=3, value=2)

    if st.sidebar.button(" DISPATCH MISSION OVER DDS", type="primary", use_container_width=True, disabled=not is_valid_point):
        ros_node.dispatch_mission(selected_robot, dest_coords, prio, sku_name)
        st.sidebar.success(f"Dispatched {selected_robot} ➔ {dest_coords}")
else:
    st.sidebar.warning("Launch ROS 2 nodes to enable dynamic dispatching.")

st.sidebar.divider()
st.sidebar.markdown("### CAD Floorplan Overlay")
uploaded_cad = st.sidebar.file_uploader("Upload Blueprint", type=["png", "jpg", "jpeg", "bmp"])
if uploaded_cad and st.sidebar.button("🔨 COMPILE CAD"):
    st.session_state.grid = parse_cad_image(uploaded_cad)
    st.session_state.cad_name = uploaded_cad.name
    st.sidebar.success("Floorplan re-compiled!")
    st.rerun()

# --- SCADA Layout ---
st.markdown(f"""
<div class="header-bar">
    <div style="font-weight:700; font-size:14px; color:#0f172a;">
        <span style="background:{'#16a34a' if available_robots else '#94a3b8'}; color:#ffffff; padding:2px 5px; border-radius:3px; font-size:10px;">
            {'ROS 2 DDS ACTIVE' if available_robots else 'AWAITING NODES'}
        </span>
        NEXUS SCADA &nbsp;|&nbsp; <span style="font-weight:400; color:#64748b; font-size:12px;">CAD: <b>{st.session_state.cad_name}</b></span>
    </div>
    <div style="font-size:11px; color:#475569;">
        CONNECTED AMRs: <b>{len(available_robots)}</b> &nbsp;•&nbsp; WMS DISPATCH: <b>ENABLED</b>
    </div>
</div>
""", unsafe_allow_html=True)

col_map, col_stats = st.columns([1.6, 1.0])

with ros_data["lock"]:
    live_fleet = {k: v.copy() for k, v in ros_data["fleet"].items()}
    live_history = list(ros_data["transport_history"])
    live_logs = list(ros_data["mesh_logs"])
    live_stats = dict(ros_data["action_point_stats"])

with col_map:
    st.markdown('<div class="panel-title">LIVE CAD FLOORPLAN & REAL-TIME AMR TRAJECTORIES</div>', unsafe_allow_html=True)
    st.plotly_chart(render_floorplan_chart(st.session_state.grid, live_fleet), use_container_width=True)
    
    st.markdown('<div class="panel-title">LIVE DECENTRALIZED P2P MESH BROADCASTS</div>', unsafe_allow_html=True)
    if live_logs:
        log_html = "".join([f"<code style='color:#0284c7; background:#ffffff; border:1px solid #e2e8f0; display:block; padding:2px 6px; margin-bottom:2px; font-size:11px;'>{l}</code>" for l in live_logs[-3:]])
        st.markdown(log_html, unsafe_allow_html=True)
    else:
        st.caption("Awaiting reservation broadcasts from active nodes...")

with col_stats:
    st.markdown('<div class="panel-title">FLEET OPERATIONAL BREAKDOWN (%) [LIVE ROS 2]</div>', unsafe_allow_html=True)
    st.markdown("""
    <div style="margin-bottom:4px;">
        <span class="legend-item"><span class="legend-box" style="background:#16a34a;"></span>Cargo</span>
        <span class="legend-item"><span class="legend-box" style="background:#22c55e;"></span>Empty</span>
        <span class="legend-item"><span class="legend-box" style="background:#eab308;"></span>Docking</span>
        <span class="legend-item"><span class="legend-box" style="background:#dc2626;"></span>Blocked</span>
    </div>
    """, unsafe_allow_html=True)
    st.plotly_chart(render_telemetry_breakdown_chart(live_fleet), use_container_width=True)
    
    st.markdown('<div class="panel-title">BATTERY STATE OF CHARGE (SoC %) [LIVE ROS 2]</div>', unsafe_allow_html=True)
    st.plotly_chart(render_battery_chart(live_fleet), use_container_width=True)
    
    st.markdown('<div class="panel-title">ACTIONPOINT SERVICE TIMES (s)</div>', unsafe_allow_html=True)
    st.plotly_chart(render_service_times_chart(live_stats), use_container_width=True)

st.markdown('<div class="panel-title">LIVE ACTIONPOINT MISSION HISTORY (ROS 2 INGRESS)</div>', unsafe_allow_html=True)
if live_history:
    st.dataframe(pd.DataFrame(live_history), use_container_width=True, height=110)
else:
    st.info("Dispatch a mission from the sidebar to record live telemetry.")

time.sleep(0.2)
st.rerun()