import PySimpleGUI as sg

sg.theme('Reddit')  # please make your windows colorful

layout = [[sg.Text('Drive robot to:'), sg.Text(size=(18,1), key='-OUTPUT-')],
            [sg.Text('Heading:'), sg.Slider(range=(-180,180),
            default_value=0,
            key='-HEAD-',
            size=(20,15),
            orientation='horizontal',
            font=('Helvetica', 12))],
            [sg.Text('Distance:'), sg.Slider(range=(0,2000),
            default_value=0,
            key='-DIST-',
            size=(20,15),
            orientation='horizontal',
            font=('Helvetica', 12))],
            [sg.Text('Rotation:'), sg.Slider(range=(-180,180),
            default_value=0,
            key='-ROT-',
            size=(20,15),
            orientation='horizontal',
            font=('Helvetica', 12))],
            [sg.Text('Speed:'), sg.Slider(range=(0,10),
            default_value=0,
            key='-SPD-',
            size=(20,15),
            orientation='horizontal',
            font=('Helvetica', 12))],
            [sg.Button('Run'), sg.Button('Exit')]]

window = sg.Window('Manual Control', layout)

while True:  # Event Loop
    event, values = window.read()
    print(event, values)
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    if event == 'Show':
        # change the "output" element to be the value of "input" element
        window['-OUTPUT-'].update(values['-IN1-'])
    

window.close()


sg.theme('Dark Blue 3')  # please make your windows colorful

layout = [[sg.Text('Rename files or folders')],
      [sg.Text('Source for Folders', size=(15, 1)), sg.InputText(), sg.FolderBrowse()],
      [sg.Text('Source for Files ', size=(15, 1)), sg.InputText(), sg.FolderBrowse()],
      [sg.Submit(), sg.Cancel()]]

window = sg.Window('Rename Files or Folders', layout)

event, values = window.read()
window.close()
folder_path, file_path = values[0], values[1]       # get the data from the values dictionary
print(folder_path, file_path)


# Choose a Theme for the Layout
sg.theme('DarkTeal9')

layout = [[sg.Text('List of InBuilt Themes')],
		[sg.Text('Please Choose a Theme to see Demo window')],
		[sg.Listbox(values = sg.theme_list(),
					size =(20, 12),
					key ='-LIST-',
					enable_events = True)],
		[sg.Button('Exit')]]

window = sg.Window('Theme List', layout)

# This is an Event Loop
while True:
	event, values = window.read()
	
	if event in (None, 'Exit'):
		break
		
	sg.theme(values['-LIST-'][0])
	sg.popup_get_text('This is {}'.format(values['-LIST-'][0]))
	
# Close
window.close()




