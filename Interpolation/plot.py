import matplotlib                                                                                        
import matplotlib.pyplot as plt                                                                          
import pandas                                                                                            
import os                                                                                                
import shutil                                                                                            
                                                                                                         
df = pandas.read_csv("/tmp/real_vs_interp", sep="\t")                                                    
df = df[df['ankle_angle_l'] <= -0.69]                                                                    
fig, ax = plt.subplots()                                                                                 
df.plot(ax=ax, x="mtp_angle_l", y="real_len", style=".-")                                                
df.plot(ax=ax, x="mtp_angle_l", y="interp_len", style=".-")                                              
fig.show()
