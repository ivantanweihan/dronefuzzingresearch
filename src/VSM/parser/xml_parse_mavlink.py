#!/usr/bin/python3

# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# This is a XML parser for ArduPilot.

import sys, getopt
import csv
import re
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, dump, ElementTree

def main(argv):
    # Parse command line arguments (i.e., input and output file)
    
    inputfile = '/home/anon/Documents/dronefuzzingresearch/src/parser/common.xml'
    outputfile = '/home/anon/Documents/dronefuzzingresearch/src/parser/output_commands.csv'

    print('Input file is "', inputfile)
    print('Output file is "', outputfile)


    if re.search(".txt", outputfile):
        output_file_type = -1
        
        store_file = open(outputfile, 'w')
        store_file.close()

        store_file = open(outputfile, 'a')

    elif re.search(".csv", outputfile):
        output_file_type = 1
        store_file = open(outputfile, 'w')
        store_file.close()

        store_file = open(outputfile, 'a')
        # creating a csv writer object
        csv_writer = csv.writer(store_file) 
        
        # writing the fields 
        fields = ['id', 'name', 'description', 
            *{str(1) + ':enumVals': '',str(1) + ':minValue': '',
            str(1) + ':maxValue': '',str(1) + ':increment': '',str(1) + ':empty/reserved': '',str(1) + ':description': '',}.keys(),
            *{str(2) + ':enumVals': '',str(2) + ':minValue': '',
            str(2) + ':maxValue': '',str(2) + ':increment': '',str(2) + ':empty/reserved': '',str(2) + ':description': '',}.keys(),
            *{str(3) + ':enumVals': '',str(3) + ':minValue': '',
            str(3) + ':maxValue': '',str(3) + ':increment': '',str(3) + ':empty/reserved': '',str(3) + ':description': '',}.keys(),
            *{str(4) + ':enumVals': '',str(4) + ':minValue': '',
            str(4) + ':maxValue': '',str(4) + ':increment': '',str(4) + ':empty/reserved': '',str(4) + ':description': '',}.keys(),
            *{str(5) + ':enumVals': '',str(5) + ':minValue': '',
            str(5) + ':maxValue': '',str(5) + ':increment': '',str(5) + ':empty/reserved': '',str(5) + ':description': '',}.keys(),
            *{str(6) + ':enumVals': '',str(6) + ':minValue': '',
            str(6) + ':maxValue': '',str(6) + ':increment': '',str(6) + ':empty/reserved': '',str(6) + ':description': '',}.keys(),
            *{str(7) + ':enumVals': '',str(7) + ':minValue': '',
            str(7) + ':maxValue': '',str(7) + ':increment': '',str(7) + ':empty/reserved': '',str(7) + ':description': '',}.keys(),
            ]
        csv_writer.writerow(fields)

    else:
        print("The output file can be either \'.txt\' or \'.csv\'.")
        sys.exit(2)



    doc = ET.parse(inputfile)

    # Load root node
    root = doc.getroot()

    enums = root.find('enums')

    commands = enums.find('enum[@name="MAV_CMD"]')

    entries = commands.findall('entry')

    for entry in entries:

        description = entry.find('description').text if entry.find('description').text != None else ''
        name = entry.get('name') if entry.get('name') != None else ''
        id = entry.get('value') if entry.get('value') != None else ''

        params = entry.findall('param')
        params_dicts = []
        for i in range(7):
            params_dicts.append({
                str(i+1) + ':enumVals': '',
                str(i+1) + ':minValue': '',
                str(i+1) + ':maxValue': '',
                str(i+1) + ':increment': '',
                str(i+1) + ':empty/reserved': 'yes',
                str(i+1) + ':description': '',
            })
            
        for param in params:
            index = param.get('index')
            params_dicts[int(index)-1][index + ':empty/reserved'] = 'no'

            enum = param.get('enum')
            params_dicts[int(index)-1][index + ':enumVals'] = ''
            
            params_dicts[int(index)-1][index + ':empty/reserved'] = 'yes' if param.text != None and ('Empty' in param.text or 'Reserved' in param.text) else 'no'
            reserved = param.get('reserved')
            if reserved == 'true':
                params_dicts[int(index)-1][index + ':empty/reserved'] = 'yes'

            if enum != None:
                params_dicts[int(index)-1][index + ':enumVals'] = '|'.join([e.get('value') for e in enums.find('enum[@name="' + enum + '"]').findall('entry')]) 
            params_dicts[int(index)-1][index + ':minValue'] = param.get('minValue') if param.get('minValue') != None else ''
            params_dicts[int(index)-1][index + ':maxValue'] = param.get('maxValue') if param.get('maxValue') != None else ''
            params_dicts[int(index)-1][index + ':increment'] = param.get('increment') if param.get('increment') != None else ''
            params_dicts[int(index)-1][index + ':description'] = param.text if param.text != None else ''

        all_param_fields = [
            *params_dicts[0].values(),
            *params_dicts[1].values(),
            *params_dicts[2].values(),
            *params_dicts[3].values(),
            *params_dicts[4].values(),
            *params_dicts[5].values(),
            *params_dicts[6].values()
        ]
        csv_writer.writerow([id, name, description, *all_param_fields])


if __name__ == "__main__":
   main(sys.argv[1:])
