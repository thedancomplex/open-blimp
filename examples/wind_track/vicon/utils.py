def parse_names(path):
    my_file = open(path, "r")
    data =  my_file.read()
    data = data.split('\n')
    return [item for item in data if len(item) > 0]
