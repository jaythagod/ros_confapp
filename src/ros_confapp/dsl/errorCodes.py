class ErrorCodes:
    def __init__(self):
        self.ecodes = [10,20,30,40,50,1000]

    def printError(self, caughtCode):
        if caughtCode == 10:
            print('Unknown command. Refer to documentation for valid command syntax')

        if caughtCode == 20:
            print("The 'ADD' command requires a corresponding 'TO' statement. Refer to documentation for further details")

        if caughtCode == 30:
            print("Insufficient parameters passed to ADD command. Refer to documentation for further details")

        if caughtCode == 40:
            print("Validation Error")

        if caughtCode == 50:
            print("Invalid command: Command length exceeds the standard. Refer to documentation for further details")

        if caughtCode == 1000:
            print("No active model loaded. Refer to documentation for further details")