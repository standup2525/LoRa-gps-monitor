from flask import Flask, jsonify, request, send_file, render_template_string, Response
import sqlite3, csv, io, os

app = Flask(__name__)
DB_PATH = "/home/fboe/main/totallogs/sensor.db"
LOG_PATH = "/home/fboe/main/totallogs"


def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

@app.route("/api/data")
def get_data():
    start = request.args.get("start")
    end = request.args.get("end")

    query = "SELECT * FROM sensor_log"
    params = []
    if start and end:
        query += " WHERE timestamp BETWEEN ? AND ?"
        params = [start, end]
    elif start:
        query += " WHERE timestamp >= ?"
        params = [start]
    elif end:
        query += " WHERE timestamp <= ?"
        params = [end]
    query += " ORDER BY timestamp ASC"

    conn = get_db()
    rows = conn.execute(query, params).fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])

@app.route("/api/summary") #필요없나? 생각해보고지우기 
def summary():
    conn = get_db()
    cur = conn.cursor()
    res = cur.execute("""
        SELECT 
            COUNT(*) AS cnt,
            AVG(uv), MAX(uv), MIN(uv),
            AVG(lux), MAX(lux), MIN(lux),
            AVG(pressure), MAX(pressure), MIN(pressure)
        FROM sensor_log
    """).fetchone()
    conn.close()

    return jsonify({
        "rows": res[0],
        "uv": {"avg": res[1], "max": res[2], "min": res[3]},
        "lux": {"avg": res[4], "max": res[5], "min": res[6]},
        "pressure": {"avg": res[7], "max": res[8], "min": res[9]}
    })

@app.route("/api/export_csv")
def export_csv():
    start = request.args.get("start")
    end = request.args.get("end")
    label = request.args.get("label")

    query = "SELECT * FROM sensor_log WHERE 1=1"
    params = []
    if start:
        query += " AND timestamp >= ?"
        params.append(start)
    if end:
        query += " AND timestamp <= ?"
        params.append(end)
    if label:
        query += " AND label = ?"
        params.append(label)

    query += " ORDER BY timestamp ASC"

    conn = get_db()
    rows = conn.execute(query, params).fetchall()
    conn.close()

    if not rows:
        return Response("No data found for given conditions.", status=404)

    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(["timestamp", "GPS", "UV", "Lux", "Pressure"])  # ✅ 필요한 컬럼만 헤더로 지정
    for r in rows:
        writer.writerow([r["timestamp"], r["gps"], r["uv"], r["lux"], r["pressure"]])
    output.seek(0)

    filename_parts = ["sensor"]
    if label:
        filename_parts.append(label)
    if start or end:
        from datetime import datetime

        def simplify(ts):
            try:
                return datetime.strptime(ts, "%Y-%m-%d %H:%M:%S").strftime("%Y%m%d")
            except:
                return ts.replace(" ", "_").replace(":", "").replace("-", "")

        if start and end:
            filename_parts.append(f"{simplify(start)}-{simplify(end)}")
        elif start:
            filename_parts.append(f"{simplify(start)}-end")
        elif end:
            filename_parts.append(f"start-{simplify(end)}")

    filename = "_".join(filename_parts) + ".csv"

    return Response(
        output,
        mimetype="text/csv",
        headers={"Content-Disposition": f"attachment; filename={filename}"}
    )


@app.route("/")
def index():
    html = open("/home/fboe/main/web/index.html", "r", encoding="utf-8").read()
    return html

@app.route("/api/label", methods=["POST"])
def save_label():
    data = request.json
    label = data.get("label")
    values = data.get("values")  

    conn = get_db()
    conn.execute("""
        INSERT INTO sensor_log
        (timestamp, gps, gps_fix, uv, lux, temperature, humidity, pressure, altitude, state, score, label)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
    """, (
        values["timestamp"], values["gps_fix"], values["gps_fix"],
        values["uv"], values["lux"], values["temperature"], values["humidity"],
        values["pressure"], values["altitude"], values["state"], values["score"],
        label
    ))
    conn.commit()
    conn.close()
    return jsonify({"status": "ok", "label": label})

@app.route("/api/label_data")
def label_data():
    label = request.args.get("label")
    conn = get_db()
    rows = conn.execute("SELECT * FROM sensor_log WHERE label = ? ORDER BY timestamp DESC", (label,)).fetchall()
    conn.close()
    return jsonify([dict(r) for r in rows])

if __name__ == "__main__":
    print("[FLASK] Dashboard server running at http://0.0.0.0:8080")
    app.run(host="0.0.0.0", port=8080)
