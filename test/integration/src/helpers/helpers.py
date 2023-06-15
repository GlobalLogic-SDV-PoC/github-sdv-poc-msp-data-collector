import os


def extract_from_dictionary(dictionary, keys_or_indexes):
    value = dictionary
    for key_or_index in keys_or_indexes:
        value = value[key_or_index]
    return value


def write_to_file(path, context):
    folder = os.path.dirname(path)
    if not os.path.exists(folder):
            os.makedirs(folder)

    context = str(context)
    f = open(path, "w")
    f.write(context)
    f.close()

    return os.path.abspath(path)
