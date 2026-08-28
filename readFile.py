def readFile(filename):
    """
    Reads numerical values and their labels from a specified file.
    Input:
        filename: Name of the file containing the data.
    Output:
        data: Dictionary containing the parsed data (label: value).
    """
    data = {}

    try:
        with open(filename, 'r') as file:
            for line in file:
                line = line.strip()
                if not line or line.startswith('%'):
                    continue  # Skip empty lines and comments

                # Split the line at the '%' symbol
                parts = line.split('%', 1)
                if len(parts) == 2:
                    value = float(parts[0].strip())
                    label_with_unit = parts[1].strip()

                    # Extract the label (remove units)
                    label = label_with_unit.split('(')[0].strip()

                    if label:  # If the label is valid
                        try:
                            data[label] = value
                        except ValueError:
                            print(f"Warning: Could not convert '{value}' to a number for label '{label}'")

    except FileNotFoundError:
        raise FileNotFoundError(f"Error: Unable to open file {filename}")

    return data