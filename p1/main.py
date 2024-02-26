import case1

if __name__ == "__main__":
    while True:
        inp = input("1 2 3 4: ")
        match inp:
            case '1':
                case1.init()
                