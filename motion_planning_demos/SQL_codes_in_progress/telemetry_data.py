# Description: This script is used to create a SQLite database to store telemetry data for the motion planning demos.

# Import the required libraries
import sys
import os
from sqlalchemy import create_engine, Column, Integer, String, Float, Sequence
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.sql import func
import numpy as np
import pdb
# Correct path to include the project's root directory
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Create a declarative base
Base = declarative_base()

# Define the classes for the tables in the database
class Waypoint(Base):
    """
    Class representing the waypoints table in the database.
    """
    __tablename__ = 'waypoints'
    id = Column(Integer, Sequence('waypoint_id_seq'), primary_key=True)
    x = Column(Float)
    y = Column(Float)
    z = Column(Float)
    
class Obstacle(Base):
    """
    Class representing the obstacles table in the database. Inherits from the Base class. 
    """
    __tablename__ = 'obstacles'
    id = Column(Integer, Sequence('obstacle_id_seq'), primary_key=True)
    x = Column(Float)
    y = Column(Float)
    z = Column(Float)
    hx = Column(Float)
    hy = Column(Float)
    hz = Column(Float)

# Create an SQLite database    
engine = create_engine('sqlite:///telemetry_data.db') # This will create a new database file called telemetry_data.db
Base.metadata.create_all(engine) # Create the tables in the database
Session = sessionmaker(bind=engine) # Create a session to interact with the database
session = Session() # Create a session object

def load_obstacles_from_csv(csv_file):
    with open(csv_file, 'r') as f:
        next(f)  # Skip the first line with GPS info
        data = np.genfromtxt(f, delimiter=',', names=True, dtype=[('posX', float), ('posY', float), ('posZ', float), ('halfSizeX', float), ('halfSizeY', float), ('halfSizeZ', float)])
    
    # Add the data to the obstacles table in the database
    for row in data:
        obstacle = Obstacle(x=row['posX'], y=row['posY'], z=row['posZ'], hx=row['halfSizeX'], hy=row['halfSizeY'], hz=row['halfSizeZ'])  # Create an Obstacle object
        session.add(obstacle)  # Add the object to the session
    session.commit()  # Commit the changes to the database


    
def get_obstacles_from_db():
    """
    Function to retrieve the obstacles from the SQLite database.
    
    Returns:
    - List of Obstacle objects
    """
    
    return session.query(Obstacle).all() # Query all the obstacles from the database

def get_waypoints_from_db():
    """
    Function to retrieve the waypoints from the SQLite database.
    
    Returns:
    - List of Waypoint objects
    """
    
    return session.query(Waypoint).all() # Query all the waypoints from the database
    
def get_obstacles_within_range(min_x, max_x, min_y, max_y, min_z, max_z):
    """
    Function to retrieve the obstacles within a specified range.
    
    Args:
    - min_x, max_x: Minimum and maximum x coordinates
    - min_y, max_y: Minimum and maximum y coordinates
    - min_z, max_z: Minimum and maximum z coordinates
    
    Returns:
    - List of Obstacle objects
    """
    
    return session.query(Obstacle).filter(
        Obstacle.x >= min_x, 
        Obstacle.x <= max_x, 
        Obstacle.y >= min_y, 
        Obstacle.y <= max_y, 
        Obstacle.z >= min_z, 
        Obstacle.z <= max_z
        ).all()

def count_obstacles():
    """
    Function to count the number of obstacles in the database.
    
    Returns:
    - Integer: Number of obstacles
    """
    
    return session.query(Obstacle).count()

def max_obstacle_height():
    """
    Function to find the maximum height of the obstacles in the database.
    
    Returns:
    - Float: Maximum height of the obstacles
    """
    
    return session.query(func.max(Obstacle.z + Obstacle.hz)).scalar()


if __name__ == "__main__":
    obstacle_csv_file = "../free_space_construction/map_data/colliders.csv"
    load_obstacles_from_csv(obstacle_csv_file) # Load the obstacles from the CSV file into the database
    print(f"Number of obstacles: {count_obstacles()}") # Print the number of obstacles in the database
    print(f"Maximum obstacle height: {max_obstacle_height()}") # Print the maximum height of the obstacles in the database
    xmin, ymin, zmin = 0, 0, 0
    xmax, ymax, zmax = 100, 100, 100
    print(f"Obstacles within range ({xmin}, {xmax}), ({ymin}, {ymax}), ({zmin}, {zmax}):")
    for obstacle in get_obstacles_within_range(xmin, xmax, ymin, ymax, zmin, zmax):
        print(obstacle.x, obstacle.y, obstacle.z, obstacle.hx, obstacle.hy, obstacle.hz)
    
   
