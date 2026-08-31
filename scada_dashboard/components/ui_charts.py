import plotly.graph_objects as go

LAYOUT_BASE = dict(
    margin=dict(l=5, r=5, t=5, b=5),
    plot_bgcolor='#ffffff',
    paper_bgcolor='#ffffff',
    font=dict(family="Inter, sans-serif", size=10, color="#475569")
)

def render_floorplan_chart(grid, fleet):
    gh, gw = len(grid), len(grid[0])
    fig = go.Figure()

    for r in range(gh):
        for c in range(gw):
            if grid[r][c] == 1:
                fig.add_shape(
                    type="rect", x0=c-0.45, y0=r-0.45, x1=c+0.45, y1=r+0.45,
                    fillcolor="#e2e8f0", line=dict(color="#94a3b8", width=1)
                )

    for r in range(gh):
        for c in range(gw):
            if grid[r][c] == 0:
                if c + 1 < gw and grid[r][c+1] == 0:
                    fig.add_trace(go.Scatter(x=[c, c+1], y=[r, r], mode='lines', line=dict(color='#bfdbfe', width=1), hoverinfo='skip', showlegend=False))
                if r + 1 < gh and grid[r+1][c] == 0:
                    fig.add_trace(go.Scatter(x=[c, c], y=[r, r+1], mode='lines', line=dict(color='#bfdbfe', width=1), hoverinfo='skip', showlegend=False))

    colors_map = {"AMR_1": "#2563eb", "AMR_2": "#059669", "AMR_3": "#7c3aed"}

    for r_id, amr in fleet.items():
        fig.add_trace(go.Scatter(
            x=[amr["goal"][1]], y=[amr["goal"][0]],
            mode='markers+text',
            marker=dict(symbol="circle-dot", size=14, color="#16a34a", line=dict(color="#14532d", width=1.5)),
            text=[f"{r_id}_DST"], textposition="top center",
            textfont=dict(size=9, color="#166534"), showlegend=False
        ))

        if amr.get("path"):
            px = [p[1] for p in amr["path"]]
            py = [p[0] for p in amr["path"]]
            fig.add_trace(go.Scatter(
                x=px, y=py, mode='lines',
                line=dict(color=colors_map.get(r_id, '#3b82f6'), width=4),
                opacity=0.4, showlegend=False, hoverinfo='skip'
            ))

        curr = amr["pos"]
        agv_color = "#dc2626" if amr["status"] == "BLOCKED" else "#15803d"
        fig.add_shape(
            type="rect",
            x0=curr[1]-0.4, y0=curr[0]-0.3, x1=curr[1]+0.4, y1=curr[0]+0.3,
            fillcolor=agv_color, line=dict(color="#0f172a", width=1.5)
        )
        fig.add_annotation(
            x=curr[1], y=curr[0], text=r_id, showarrow=False,
            font=dict(color="#ffffff", size=9, family="Inter")
        )

    fig.update_layout(
        **LAYOUT_BASE,
        xaxis=dict(range=[-0.5, gw - 0.5], showgrid=False, zeroline=False, showticklabels=False),
        yaxis=dict(range=[gh - 0.5, -0.5], showgrid=False, zeroline=False, showticklabels=False),
        height=440
    )
    return fig

def render_telemetry_breakdown_chart(fleet):
    robots = list(fleet.keys())
    c_cargo, c_empty, c_docking, c_blocked = [], [], [], []

    for r in robots:
        t = fleet[r]["ticks"]
        tot = max(1, sum(t.values()))
        c_cargo.append(round((t["cargo"] / tot) * 100, 1))
        c_empty.append(round((t["empty"] / tot) * 100, 1))
        c_docking.append(round((t["docking"] / tot) * 100, 1))
        c_blocked.append(round((t["blocked"] / tot) * 100, 1))

    fig = go.Figure()
    fig.add_trace(go.Bar(y=robots, x=c_cargo, orientation='h', name='Cargo', marker=dict(color='#16a34a')))
    fig.add_trace(go.Bar(y=robots, x=c_empty, orientation='h', name='Empty', marker=dict(color='#22c55e')))
    fig.add_trace(go.Bar(y=robots, x=c_docking, orientation='h', name='Docking', marker=dict(color='#eab308')))
    fig.add_trace(go.Bar(y=robots, x=c_blocked, orientation='h', name='Blocked', marker=dict(color='#dc2626')))
    
    fig.update_layout(
        **LAYOUT_BASE,
        barmode='stack', height=110,
        xaxis=dict(range=[0, 100], showgrid=True, gridcolor="#f1f5f9"),
        yaxis=dict(tickfont=dict(size=10)),
        showlegend=False
    )
    return fig

def render_battery_chart(fleet):
    robots = list(fleet.keys())
    bat_vals = [fleet[r]["battery"] for r in robots]
    fig = go.Figure()
    fig.add_trace(go.Bar(
        y=robots, x=bat_vals, orientation='h',
        marker=dict(color=bat_vals, colorscale=[[0, '#ef4444'], [0.5, '#eab308'], [1.0, '#16a34a']], cmin=0, cmax=100, showscale=False),
        text=[f"{v:.1f}%" for v in bat_vals], textposition="inside",
        textfont=dict(color="#ffffff", size=9)
    ))
    fig.update_layout(
        **LAYOUT_BASE,
        height=100,
        xaxis=dict(range=[0, 100], showgrid=True, gridcolor="#f1f5f9"),
        yaxis=dict(tickfont=dict(size=10))
    )
    return fig

def render_service_times_chart(action_point_stats):
    pts = list(action_point_stats.keys())
    measured = list(action_point_stats.values())
    fig = go.Figure()
    fig.add_trace(go.Bar(y=pts, x=measured, orientation='h', marker=dict(color='#38bdf8')))
    fig.update_layout(
        **LAYOUT_BASE,
        height=100,
        xaxis=dict(showgrid=True, gridcolor="#f1f5f9"),
        yaxis=dict(tickfont=dict(size=9)),
        showlegend=False
    )
    return fig