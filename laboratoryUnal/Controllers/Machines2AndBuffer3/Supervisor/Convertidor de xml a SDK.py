# Import BeautifulSoup
from bs4 import BeautifulSoup as bs
from tkinter import Tk
from tkinter.filedialog import askopenfilename
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
#from xml2sdk import xml2sdk
content = []
events = ''
states = ''
stateNames = []
transitions = ''
eventLabel = ''
eventId = ''
AddString = ''
initialState = ''
noncontrollableEvents = []
factoryIoFileName = ''
XmlFileName = ''
csFileName = ''
# Read the XML file


def xml2sdk(XmlFileName):
    #with open("supervisorBuffer2.xml", "r") as file:
    with open(XmlFileName, "r") as file:
        # Read each line in the file, readlines() returns a list of lines
        content = file.readlines()
        # Combine the lines in the list into a string
        content = "".join(content)
        bs_content = bs(content, "lxml") 

    events = bs_content.find_all("event")
    states = bs_content.find_all("state")
    transitions = bs_content.find_all("transition")

    for state in states:
        nombre = state["name"]
        try: 
            state["initial"]
            initialState = state["id"]
        except:
            continue
        stateNames.append(nombre)
    
        
    factoryIoFileName = ''    
    for letter in XmlFileName:
        if letter == "/":
            factoryIoFileName = ''
        elif letter == ".":
            break
        else:
            factoryIoFileName = factoryIoFileName + letter
    csFileName = factoryIoFileName + 'Supervisor.cs'
    try:
        f = open(csFileName, "x")
    except:
        f = open(csFileName, "w")

    ##############  PROGRAM WRITING STARTS  ########################
    #Headers
    f.write("using System;\n")
    f.write("using System.Collections.Generic;\n")
    f.write("using System.Threading;\n\n")

    #namespace Controllers - class 
    f.write("namespace Controllers.Scenes\n")
    f.write("{\n")
    stringy = "    class " + factoryIoFileName + "Supervisor\n"
    f.write(stringy)
    f.write("    {\n\n")
    f.write("        // #### VARIABLE CREATION TO ALLOCATE IN MEMORY ####\n")
    f.write("        private int currentState;\n")
    f.write("        private int evento;\n")
    f.write("        private Dictionary<(int, int), int> transiciones;\n")
    f.write("        private Dictionary<string, int> eventLabels;\n\n")
    f.write("        private Dictionary<int, (string, string)> eventLabelsInverse;\n\n")
    f.write("        private Dictionary<int, string> stateLabels;\n\n")
    f.write("        public void CreateController()\n")
    f.write("        {\n")
    f.write("            transiciones = new Dictionary<(int, int), int>();\n")
    f.write("            eventLabels = new Dictionary<string, int>();\n")
    f.write("            eventLabelsInverse = new Dictionary<int, (string, string)>();\n")
    f.write("            stateLabels = new Dictionary<int, string>();\n\n")

    f.write("            currentState = " + initialState + ";\n") 
    f.write("            //#########  TRANSICIONES START ############\n\n")


    for transition in transitions:
        AddString ='            ' + 'transiciones.Add((' + transition["source"] + ', ' + transition["event"] + '), ' + transition["dest"] + ');\n'
        f.write(AddString)
    f.write("\n            //#########  TRANSICIONES END ############\n\n")
    f.write("\n            //#########  STATELABEL START ############\n\n")
    for state in states:
        AddString = '            ' + 'stateLabels.Add(' + state["id"] + ', "' + state["name"] + '");\n'
        f.write(AddString)

    f.write("\n            //#########  STATELABEL END ############\n\n")
    f.write("            //#########  EVENTLABEL START ############\n\n")

    for event in events:    
        AddString = '            ' + 'eventLabels.Add("' + event["label"] + '", ' + event["id"] + ');\n'
        f.write(AddString)
        try:
            event["controllable"]
            noncontrollableEvents.append(event["id"])
        except:
            continue
    f.write("\n")
    for event in events:    
        try:
            event["controllable"]
            AddString = '            ' + 'eventLabelsInverse.Add(' + event["id"] + ', ("' + event["label"] + '", "nc"));\n'
            f.write(AddString)
        except:
            AddString = '            ' + 'eventLabelsInverse.Add(' + event["id"] + ', ("' + event["label"] + '", "c"));\n'
            f.write(AddString)
    f.write("\n            //#########  EVENTLABEL END ############\n\n")
    stringy = '            Console.WriteLine("' + r'\n' + 'Current state is: " + stateLabels[currentState] + ' + r'"\n"' + ');\n'
    f.write(stringy)
    stringy = '            Console.WriteLine("' + 'List of active events. Choose one and press enter: ' + r'\n' + '");\n'
    f.write(stringy)
    f.write("            for (int i = 0; i < eventLabels.Count; i++)\n")
    f.write('            {\n')
    f.write('                if (transiciones.ContainsKey((currentState, i)) && eventLabelsInverse[i].Item2 == "c")\n')
    f.write('                {\n')
    stringy = '                    Console.WriteLine(i + ": " + eventLabelsInverse[i].Item1 + "' + r'\n' + '");\n'
    f.write(stringy)
    f.write('                }\n')
    f.write('            }\n')
    stringy = '            Console.WriteLine("Type event number and press enter to execute or press button on Factory I/O interface:' + r'\n' + '");'
    f.write(stringy)
    f.write("        }\n\n")
    f.write("        public bool IsInActiveEventsLights(string newState)\n")
    f.write("        {\n")
    f.write('            int newStateInt = eventLabels[newState];\n')
    f.write('            if (transiciones.ContainsKey((currentState, newStateInt)))\n')
    f.write('            {\n')
    f.write('                return (true);\n')
    f.write('            }\n')
    f.write('            else\n')
    f.write('            {\n')
    f.write('                return (false);\n')
    f.write('            }\n')
    f.write("        }\n\n")
    f.write("        public bool IsInActiveEvents(int newState)\n")
    f.write("        {\n")
    f.write('            if (transiciones.ContainsKey((currentState, newState)) && eventLabelsInverse[newState].Item2 == "c")\n')
    f.write('            {\n')
    f.write('                return (true);\n')
    f.write('            }\n')
    f.write('            else\n')
    f.write('            {\n')
    f.write('                return (false);\n')
    f.write('            }\n')
    f.write("        }\n\n")
    f.write("        public void ListOfActiveEvents()\n")
    f.write("        {\n")
    stringy = '            Console.WriteLine("----------------------------------------' + r'\n' + '");\n'
    f.write(stringy)
    stringy = '            Console.WriteLine("' + 'List of active events. Choose one and press enter or wait:' + r'\n' + '");\n\n'
    f.write(stringy)
    f.write('            for (int i = 0; i < eventLabels.Count; i++)\n')
    f.write('            {\n')
    f.write('                if (transiciones.ContainsKey((currentState, i)) && eventLabelsInverse[i].Item2 == "c")\n')
    f.write('                {\n')
    f.write('                    Console.WriteLine(i + ": " + eventLabelsInverse[i].Item1);\n')
    f.write('                }\n')
    f.write('            }\n')
    stringy = '            Console.WriteLine("' + r'\n' + '----------------------------------------");\n'
    f.write(stringy)
    f.write("        }\n\n")
    f.write("        public string StateName(int eventNumber)\n")
    f.write("        {\n")
    f.write("            if (eventLabelsInverse.ContainsKey(eventNumber))\n")
    f.write("            {\n")
    f.write("                return (eventLabelsInverse[eventNumber].Item1);\n")
    f.write("            }\n")
    f.write("            else\n")
    f.write("            {\n")
    f.write('                Console.WriteLine("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");\n')
    stringy = ('                Console.WriteLine("' + r'\n' + 'Event number pressed does not exist. Try again.' + r'\n' + '");\n')
    f.write(stringy)
    f.write('                Console.WriteLine("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");\n')
    f.write('                ListOfActiveEvents();\n')
    f.write('                return ("Event number pressed does not exist");\n')
    f.write("            }\n")
    f.write("        }\n\n")
    f.write("        public bool On(string eventoLabel)\n")
    f.write("        {\n")
    f.write("            evento = eventLabels[eventoLabel];\n")
    f.write("            if (transiciones.ContainsKey((currentState, evento)))\n")
    f.write("            {\n")
    f.write("                currentState = transiciones[(currentState, evento)];\n")

    if bool(noncontrollableEvents):
        stringy = "                if ("
        for count,event in enumerate(noncontrollableEvents):
            stringy = stringy + "evento != " + event
            if count != len(noncontrollableEvents) - 1:            
                stringy = stringy + " && "
        stringy = stringy + ")\n"
        f.write(stringy)
        f.write('                {\n')
        stringy = '                    Console.WriteLine("oooooooooooooooooooooooooooooooooooooooo' + r'\n' + '");\n'
        f.write(stringy)
        f.write('                    Console.WriteLine(eventoLabel + " event approved");\n')
        f.write('                    Console.WriteLine("Current state is: " + stateLabels[currentState]);\n')
        stringy = '                    Console.WriteLine("' + r'\n' + 'oooooooooooooooooooooooooooooooooooooooo");\n'
        f.write(stringy)
        f.write('                    ListOfActiveEvents();\n')
        f.write('                }\n')

        f.write('                else\n')
        f.write('                {\n')
        stringy = '                    Console.WriteLine("oooooooooooooooooooooooooooooooooooooooo' + r'\n' + '");\n'
        f.write(stringy)
        f.write('                    Console.WriteLine(eventoLabel + " event is uncontrollable and must be enabled");\n')
        f.write('                    Console.WriteLine("Current state is: " + stateLabels[currentState]);\n')
        stringy = '                    Console.WriteLine("' + r'\n' + 'oooooooooooooooooooooooooooooooooooooooo");\n'
        f.write(stringy)
        f.write('                    ListOfActiveEvents();\n')
        f.write('                }\n')
    else:
        stringy = '                    Console.WriteLine("oooooooooooooooooooooooooooooooooooooooo' + r'\n' + '");\n'
        f.write(stringy)
        f.write('                    Console.WriteLine(eventoLabel + " event approved");\n')
        f.write('                    Console.WriteLine("Current state is: " + stateLabels[currentState]);\n')
        stringy = '                    Console.WriteLine("' + r'\n' + 'oooooooooooooooooooooooooooooooooooooooo");\n'
        f.write(stringy)
        f.write('                    ListOfActiveEvents();\n')

    f.write('                return true;\n')
    f.write('            } else\n')
    f.write('            {\n')
    f.write('                Console.WriteLine(eventoLabel + " event blocked");\n')
    f.write('                Thread.Sleep(800);\n')
    f.write('                return false;\n')
    f.write('            }\n')
    f.write('        }\n')
    f.write('    }\n')
    f.write('}')
    f.close()
        
    print("\n\nFile named " + csFileName + " created succesfully.\n\n")


def get_file():
  filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
  if not filename.endswith('.xml'):
     messagebox.showerror("Error", "Must select an XML file")
  else:
     file_to_convert.config(text=filename)
     run_button.config(state="normal")

def run_xml_to_sdk():
  filename = file_to_convert.cget("text")
  print("Running xml2Convert", filename)
  xml2sdk(filename)



if __name__ == '__main__':
    root = Tk()
    frm = ttk.Frame(root, padding=10)
    frm.grid()
    ttk.Label(frm, text="Select an .xml file to run xml2SDK: ").grid(column=0, row=0)
    ttk.Button(frm, text="Open", command=get_file).grid(column=1, row=0)

    file_to_convert = ttk.Label(frm, text="")
    file_to_convert.grid(column=0, row=1)

    run_button = ttk.Button(frm, text="Run xml2SDK", command=run_xml_to_sdk)
    run_button.config(state="disabled")
    run_button.grid(column=1, row=1)
    #Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    root.mainloop()