#!/usr/bin/python3

"""
  File:   nist_nav_test_report.py
  Author: Daniel Lynch
  Task:   Generate summary statistics and charts for NIST Industrial
  reporting.

  Requires openpyxl - https://openpyxl.readthedocs.io/en/stable/
 
  log data my_data.csv
 
  Usage: python3 nist_nav_test_report.py my_data.csv

  Outputs new dir "results" with the following additional items:

        my_data/
            my_data.xlsx [includes summary + raw data]
            figs/
                position.xlsx and .jpeg
                results_bar.xlsx
                    
"""
from sys import argv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pathlib
from datetime import datetime

# set chart style
matplotlib.style.use('ggplot')


class Reporter:

    def __init__(self, my_data):
        self.data_name = my_data
        self.df_raw, self.df_clean = self.load_data()
        self.full_name = self.get_report_name()
        self.fig_dir = ""

    def run(self):
        self.make_directories()

        self.get_position_chart()

        highlights = self.get_highlights()
        time_to_goal_deltas, mu, sigma = self.get_time()
        self.plot_time_histogram(time_to_goal_deltas, mu, sigma)
        self.get_results_in_excel(highlights, time_to_goal_deltas)
        print("Ran!")
        
            
    def get_report_name(self):
        # returns reportname as string
        # get date timestamp
        x = datetime.now()
        # => "Tue_Nov_3_10:44:22_2020"
        now = x.strftime("%c").replace("  "," ").replace(" ","_")
        # eg "sim_log.csv" => "sim_log"
        report_name = self.data_name.replace(".csv","")
        # => "sim_log_Tue_Nov_3_10:44:22_2020"
        return f"{report_name}_{now}"
    
    def make_directories(self):
        # create dir for given report
        # plus child dir for charts
        dir_w_figs = f"./{self.full_name}/figs"
        self.fig_dir = dir_w_figs
        pathlib.Path(dir_w_figs).mkdir(parents=True, exist_ok=True)

    def load_data(self):
        # read csv data into dataframe object
        raw_df = pd.read_csv(self.data_name)
        clean_df = raw_df.copy()
        # replace collision's null values w/ 0
        clean_df.replace({'field.collision.x':-100000, 'field.collision.y':-100000},0,inplace=True)
        #remove unnecessary cols
        clean_df=clean_df.drop(['field.header.frame_id'],axis=1)
        return raw_df, clean_df

    def get_position_chart(self):
        ### create robot position chart
        # create new df for plotting robot's path
        df_pos = self.df_clean[['field.robot_pos.x','field.robot_pos.y']].copy()
        df_pos = df_pos.rename(columns={'field.robot_pos.x':'robot_pos.x','field.robot_pos.y':'robot_pos.y'})
        # plot robot's path
        robot_path = df_pos.plot.scatter(x='robot_pos.x',
                                   y='robot_pos.y',
                                   #xlim=(-1,6.8),
                                   #ylim=(-1.0,13.00),
                                   figsize=(20,20),
                                   s=1,
                                   c='cornflowerblue')

        # save
        path_fig = robot_path.get_figure()
        path_fig.savefig(f"{self.fig_dir}/robot_position_graph.png")
        plt.close(path_fig)

    def get_highlights(self):
        ### get highlights metrics: successes, attempts, collisions
        # note that in the logger, 1 iteration means 2 routes: a->b & b->a
        # copy df
        #https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.DataFrame.copy.html
        df = self.df_clean.copy()
        number_of_attempts = df['field.iteration'].unique().size #* 2 
        num_successful_routes = df['field.event'][df['field.event'].str.contains('Goal was reached', na=False)].size
        route_success_rate = (num_successful_routes / number_of_attempts)*100
        # get # collisions
        number_of_collisions = df['field.collision.x'].unique().size - 1
        highlights = { 'Label': ['Route Attempts','Successful Routes', 'Route Success Rate', 'Collisions'], 'Value': [number_of_attempts,num_successful_routes,route_success_rate,number_of_collisions] }
        highlights_df = pd.DataFrame(highlights)
        return highlights_df


    def get_time(self,min_time_req=8):
        """ min_time_req : sometimes the robot idles in between runs;
            this is a threshold for the minimum time in seconds required to be considered a run
            by default we discard anything less than 8 seconds
        """
        ### time metrics: time summary stats, max route time, min route time
        # get summary stats of route times
        #split rows into "Goal x registered... Goal was reached" chunks
        dfsub = self.df_clean[['field.header.stamp','field.iteration','field.event']].copy()
        df_goals = dfsub[dfsub['field.event'].str.contains('Goal', na=False)]
        #df_goals['field.header.stamp'].diff()
        df_goal_times = pd.to_datetime(df_goals['field.header.stamp'])
        diffs = df_goal_times.diff().dropna()
        diffs.apply(lambda x: pd.Timedelta(x) if not pd.isnull(x) else pd.Timedelta(0, unit='ns'))
        time_to_goal_deltas = pd.DataFrame(diffs)
        #drop first row
        time_to_goal_deltas = time_to_goal_deltas[time_to_goal_deltas > pd.Timedelta(seconds=2)].dropna()
        # mean and std of route times
        mu = time_to_goal_deltas['field.header.stamp'].mean().seconds
        sigma = time_to_goal_deltas['field.header.stamp'].std().seconds
        return time_to_goal_deltas, mu, sigma

    def plot_time_histogram(self,time_to_goal_df, mu, sigma):
        
        # plot histogram of route times
        time_to_goal = time_to_goal_df['field.header.stamp'].astype(
            'timedelta64[s]').plot.hist( ) #xlim=(55,70), ylim=(0,40), figsize=(10,5)
        # comment out below to remove inserting mu,sigma in graph
        plt.text(68,30, r"$\mu={0}$, $\sigma={1}$".format(mu,sigma),horizontalalignment='right', fontsize=16)
        plt.title('Distribution of Time to Goal per Run (s)', fontsize=18)
        plt.xlabel('Time to Goal per Run (s)', fontsize=15)
        plt.ylabel('Number of Runs',fontsize=15)
        # save
        #time_to_goal.get_figure().savefig("{0}/time_to_goal_per_route_histogram.png".format(dir_w_figs))
        plt.savefig(f"{self.fig_dir}/time_to_goal_per_run_histogram.png")

    def get_results_in_excel(self,highlights_df, time_to_goal_df):
        ### create excel file w/ summary stats and raw data
        # write to .xlsx file
        with pd.ExcelWriter(f"./{self.full_name}/{self.full_name}.xlsx") as writer:
            highlights_df.to_excel(writer, sheet_name='Summary', index=False)
            time_to_goal_df.describe().to_excel(writer, sheet_name='Summary', startcol=3)
            self.df_raw.to_excel(writer, sheet_name='Raw_Data', index=False)



if __name__=='__main__':

    #r = Reporter(argv[1]) # get data as cli arg
    r = Reporter("nerve_physical_2_aisles_blocked.csv")
    r.run()