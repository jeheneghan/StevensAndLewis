#Stevens and Lewis

F16 and Longitudinal Tranport Simulation Environment and Sandbox

Python 3.9.7 from https://www.python.org/downloads/windows/
Check the box to Add Python to PATH when installing

Download Visual Studio if you do not already have it https://visualstudio.microsoft.com/downloads/
I use visual studio code for my development environment, which you can download from the same link

Suggest creating your own virtual environment by running:
python -m venv .venv
.venv\Scripts\activate

Install packages by running:
pip install -r requirements.txt

List of packages:
numpy
control
keyboard
pygame
notebook
ipympl
matplotlib

Run main.py in the debugger to confirm everything works as expected

press 'e' to exit
press 't' to create a new trim state and modify other settings

Hardware:
I use the Thrustmaster T-Flight Hotas, but any joystick should work for elevator and aileron control
link: https://www.amazon.com/dp/B07643TW2V?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_2

The system will automatically detect a joystick
If no joystick is found use step inputs on the keyboard:
'w' - pitch up
's' - pitch down
'd' - roll right
'a' - roll left
'left arrow' - yaw left
'right arrow' - yaw right
'up arrow' - throttle up
'dn arrow' - throttle down

Examples:
All examples use jupyternotebook .ipynb files
You can run these using jupyter's web based IDE or directly in VS Code
tip: if you have trouble viewing an output, try changing presentation to Markdown