from thefuzz import fuzz
from thefuzz import process



def matchLabels(place, roomLabels):

    #print(place)


    matches = process.extract(place, roomLabels, limit=3)
    matches = set(matches)
    matches = sorted(matches, key=lambda tup: tup[1], reverse=True)

    # matchesIDandProb = []
    # for match in matches:
    #     roomIds = [index for index, value in enumerate(roomLabels) if value == match[0]]
    #     for rid in roomIds:
    #         matchesIDandProb.append([rid, match[0], match[1]])

    # print(matches)
    # print(type(matches))

    return matches