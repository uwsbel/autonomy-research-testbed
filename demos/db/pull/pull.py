# Import some classes from the miniav module
from miniav.db import MiniAVDatabase
from miniav.utils.logger import LOGGER, set_verbosity

# Other imports
import os

# Set the verbosity to log everything
set_verbosity(2)

# Instantiate the database
db = MiniAVDatabase("data")

# Push a local file to the database
ls = db.ls()
bag = db.pull(ls[0])

# Make sure it is present in the current working directory
print(bag.db_name in os.listdir('.'))
