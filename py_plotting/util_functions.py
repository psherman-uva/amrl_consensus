#!/usr/bin/env python3

"""
File:   util_functions.py
Author: psherman-uva
Date:   July 2024
"""

import sqlite3

def execute_cmd(db_filename, sql_cmd):

  conn = sqlite3.connect(db_filename)
  
  c = conn.cursor()
  res = c.execute(sql_cmd)

  conn.close()

  return res

def read_data_from_database(db_filename, sql_cmd):
  conn = sqlite3.connect(db_filename)
  conn.row_factory = sqlite3.Row
  
  c = conn.cursor()
  c.execute(sql_cmd)
  
  data = {}
  for d in c.description: 
    data[d[0]] = []

  for row in c:
    for key in data.keys():
      data[key].append(row[key])

  conn.close()
  return data

def read_all_data_from_table(db_filename, table):
  sql_cmd = f"SELECT * FROM {table}"
  return read_data_from_database(db_filename, sql_cmd)

def read_all_formation_data(db_filename, table, formation):
  sql_cmd = f"SELECT * FROM {table} WHERE formation='{formation}'"
  return read_data_from_database(db_filename, sql_cmd)

def delete_obs_file_data(db_filename, table, obs_file):
  conn = sqlite3.connect(db_filename)
  c = conn.cursor()

  sql_cmd = f"DELETE FROM {table} WHERE obs_file='{obs_file}'"
  print(sql_cmd)
  c.execute(sql_cmd)

  conn.commit()
  conn.close()