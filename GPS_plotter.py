"""
File: GPSPlotter.py
Date: 21/01/2025
Version: 1.0
Description:
    This script contains the GPSPlotter class for plotting GPS data in real-time.
Modules:
    matplotlib.pyplot: For plotting GPS data on a map.
    cartopy.crs: For specifying the map projection.
    cartopy.feature: For adding features to the map.
    time: For pausing the plot to update the display.
Usage:
    Import the GPSPlotter class and use it to plot GPS data in real-time.
"""
import matplotlib.pyplot as plt
import cartopy.crs as ccrs
import cartopy.feature as cfeature
import time

class GPSPlotter:
    def __init__(self, area=[151.231, 151.2325, -33.9185, -33.9175], add_feature=False):
        """
        Initialize the GPSPlotter class with the specified map extent.

        Parameters:
            area (list): The map extent in the format [lon_min, lon_max, lat_min, lat_max].
                         Default is [151.231, 151.2325, -33.9185, -33.9175] for UNSW test area.
                         Use [140, 154, -40, -28] for MGA2020 zone 56.
            add_feature (bool): Whether to add features to the map. Default is False.
        """
        self.fig = plt.figure(figsize=(10, 5))
        self.ax = plt.axes(projection=ccrs.PlateCarree())

        self.ax.set_extent(area, crs=ccrs.PlateCarree())

        # Add features to the map within the specified extent
        if add_feature:
            self.ax.add_feature(cfeature.LAND, edgecolor='black')
            self.ax.add_feature(cfeature.OCEAN)
            self.ax.add_feature(cfeature.COASTLINE)
            self.ax.add_feature(cfeature.BORDERS, linestyle=':')

        self.ax.gridlines(draw_labels=True)
        self.last_lon, self.last_lat = None, None

    def plot(self, lat, lon):
        """
        Plot the GPS data on the map.

        Parameters:
            lat (float): The latitude of the GPS coordinate.
            lon (float): The longitude of the GPS coordinate.
        """
        try:
            if self.last_lon is not None and self.last_lat is not None:
                self.ax.plot([self.last_lon, lon], [self.last_lat, lat], color='red', transform=ccrs.Geodetic())
            self.ax.plot(lon, lat, marker='o', color='red', markersize=1, transform=ccrs.Geodetic())
            self.last_lon, self.last_lat = lon, lat

            plt.draw()
            plt.pause(0.1)
        except ValueError:
            print(f"Invalid data: {data}")

    def close(self):
        plt.close()

    def show(self):
        plt.show()

    def save(self, filename):
        plt.savefig(filename)