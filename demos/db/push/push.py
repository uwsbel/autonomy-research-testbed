# Import some classes from the miniav module
from miniav.db import MiniAVDatabase
from miniav.utils.logger import LOGGER, set_verbosity

# Set the verbosity to log everything
set_verbosity(2)

# Instantiate the database
db = MiniAVDatabase("data")

# Push a local file to the database
db.push("ros1.bag")

# Check to make sure it is really there
ls = db.ls()
print(ls)
