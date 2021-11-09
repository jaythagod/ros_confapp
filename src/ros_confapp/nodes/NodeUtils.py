class NodeUtils():
    def checkNodeExistence(self, needle, nodeList):
        for node in nodeList:
            if node ==  needle:
                return "live"
