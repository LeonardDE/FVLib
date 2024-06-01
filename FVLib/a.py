import json

f = open("output.json")
data = json.load(f)
f.close()

print([inst["t"] for inst in data["FVs"][0]["dynamic_data"] if len(inst["new_plan"]) > 0])
