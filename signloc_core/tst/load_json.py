import numpy as np

def parseActionDist(actionString):

	#actionDict = {"RIGHT": 0, "STRAIGHT":1, "LEFT":2, "BACK":3, "LOCATIONAL":4, "STRAIGHT-RIGHT":5, "STRAIGHT-LEFT": 6}
	actionDict = {"RIGHT": 0, "STRAIGHT-RIGHT":1, "STRAIGHT":2, "STRAIGHT-LEFT":3, "LEFT":4, "BACK-LEFT":5, "BACK":6, "BACK-RIGHT":7, "LOCATIONAL":8, "UP":9, "DOWN":10}
	actionDist = []

	for key, value in actionString.items():
		
		actionDist.append(np.array([actionDict[key], value]))

	actionDist = np.asarray(actionDist)

	return actionDist

# counter clock wise 0 is right, then straight , back, left
def action2rad(action, actionNum):

	return -0.5 * np.pi + action * 2 * np.pi / actionNum


def loadGT(jsonPath):

	import json
	actionDict = {"RIGHT": 0, "STRAIGHT-RIGHT":1, "STRAIGHT":2, "STRAIGHT-LEFT":3, "LEFT":4, "BACK-LEFT":5, "BACK":6, "BACK-RIGHT":7, "LOCATIONAL":8, "UP":9, "DOWN":10}


	with open(jsonPath) as f:
		jsonGT = json.load(f)

	gts = {}

	for item in jsonGT:

		if "gt" in item:
			gt = item["gt"]
			gt[3] = actionDict[gt[3]]
			gt[3] = action2rad(gt[3], 8)
			gts[item["frame_path"]] = gt

	return gts

def loadPredictions(jsonPath):

	import json

	with open(jsonPath) as f:
		jsonGT = json.load(f)

	predictions = {}

	for item in jsonGT:

		json_string = item["dircn_distribution"]
		json_string = json_string.replace("\"", '')
		json_string = json_string.replace("'", '\"')

		try:
			pdict = {}
			pred = json.loads(json_string)
			for place, actionString in pred.items():
				if "LOCATIONAL" in actionString:
					continue
				pdict[place] = parseActionDist(actionString)

			predictions[item["frame_path"]] = pdict
		except:
			print("failed to load " + item["frame_path"])
		
	return predictions