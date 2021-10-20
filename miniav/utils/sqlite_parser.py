"""
File contains helper classes/methods to parse and create sqlite database files
"""

# External library imports
import sqlite3


class SQLiteHelper:
    """
    Helper class to mess with sqlite3 files

    Args:
        filename (str): The file that will be read from and/or written to.
    """

    def __init__(self, filename: str):
        self._filename = filename
        self._conn = None

    def execute(self, sql, *args):
        """
        Execute a sql command. Will grab the cursor and execute the command

        Returns:
            sqlite3.Cursor: The cursor instance of the connected file
        """
        try:
            c = self._conn.cursor()
            c.execute(sql, *args)
            return c
        except sqlite3.Error as e:
            print(e)
            return None

    def connect(self):
        """
        Connect to the database file.
        """
        try:
            self._conn = sqlite3.connect(self._filename)
        except sqlite3.Error as e:
            print(e)

    def __enter__(self):
        """
        Used in conjunction with the 'with'/'as' syntax.

        When using 'with'/'as', __enter__ is called when entering the block. The
        database will be automatically connected to.
        """
        self.connect()
        return self

    def __exit__(self, type, value, traceback):
        """
        Used in conjunction with the 'with'/'as' syntax.

        When using 'with'/'as', __exit__ is called when exiting the block. The
        database will be automatically committed (saved) and closed.
        """
        if self._conn:
            self._conn.commit()
            self._conn.close()
