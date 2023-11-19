'''
  Module to route all GPIO statuses.
'''

# Import required libraries
from flask import Flask, render_template, Blueprint


mode_page = Blueprint('modeone', __name__)
@mode_page.route('/mode/', methods=['GET', 'POST'])
def mode_oneindex():
  return render_template('mode.html')