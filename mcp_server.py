"""MCP stdio server for PID Controller."""
import sys
import json

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

from client import PIDController

controller = PIDController()

def handle_rpc(request):
    req_id = request.get("id")
    method = request.get("method")
    params = request.get("params", {})

    if method == "tools/list":
        return {
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "tools": [
                    {
                        "name": "configure_pid",
                        "description": "Configure PID gains and output limits",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "kp": {"type": "number"},
                                "ki": {"type": "number"},
                                "kd": {"type": "number"},
                                "min_out": {"type": "number"},
                                "max_out": {"type": "number"},
                                "filter_alpha": {"type": "number"}
                            },
                            "required": ["kp", "ki", "kd"]
                        }
                    },
                    {
                        "name": "step_pid",
                        "description": "Execute one control step for given setpoint and measurement",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "setpoint": {"type": "number"},
                                "measured_value": {"type": "number"},
                                "dt": {"type": "number"}
                            },
                            "required": ["setpoint", "measured_value", "dt"]
                        }
                    }
                ]
            }
        }
    elif method == "tools/call":
        name = params.get("name")
        args = params.get("arguments", {})
        if name == "configure_pid":
            controller.kp = float(args.get("kp", 1.0))
            controller.ki = float(args.get("ki", 0.0))
            controller.kd = float(args.get("kd", 0.0))
            min_o = float(args.get("min_out", -100.0))
            max_o = float(args.get("max_out", 100.0))
            controller.min_out, controller.max_out = min_o, max_o
            controller.filter_alpha = float(args.get("filter_alpha", 0.1))
            controller.reset()
            return {"jsonrpc": "2.0", "id": req_id, "result": {"status": "configured"}}
        elif name == "step_pid":
            sp = float(args.get("setpoint", 0.0))
            mv = float(args.get("measured_value", 0.0))
            dt = float(args.get("dt", 0.01))
            out = controller.compute(sp, mv, dt)
            return {"jsonrpc": "2.0", "id": req_id, "result": {"output": out, "error": sp - mv, "integral": controller.integral}}
        return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32601, "message": f"Method {name} not found"}}
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": -32600, "message": "Invalid request"}}

def main():
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            req = json.loads(line)
            res = handle_rpc(req)
            sys.stdout.write(json.dumps(res) + "\n")
            sys.stdout.flush()
        except Exception as e:
            sys.stdout.write(json.dumps({"jsonrpc": "2.0", "error": {"code": -32700, "message": str(e)}}) + "\n")
            sys.stdout.flush()

if __name__ == "__main__":
    main()
