library(ggplot2)
library(reshape2)
library(plyr)
#rootDir <- "/Users/adrian/git/point-pair-features/samples/Bunny_RealData"
#rootDir <- "/Users/adrian/git/point-pair-features/samples/Bunny_Sphere/knn"
rootDir <- "/Users/adrian/git/point-pair-features/samples/Bunny_Sphere/nFrames"



w<-6
h<-4




resultsFile <- paste(rootDir, "results_timings.txt",sep="/");
#dir <- "/Users/adrian/git/point-pair-features/build/allTimings.txt"
dat <- read.csv(resultsFile)
#dat<-transform(dat,totalTime=X0+A+B+C)

accuracy<-dat[,c("params","RPE.C","ATE.C")]#"totalTime")]
accuracy<-rename(accuracy, c("RPE.C"="RPE", "ATE.C"="ATE"))


timings<-dat[,c("params","X0","A","B","C")]
timings<-rename(timings, c("X0"="0"))#, "A"="pairwise alignment","B"="pairwise refinement", "C"="multiview refinement"))


plotAccuracy <- function(data){
  df_melted <- melt(data,variable.name="metric")
  df_meaned <- ddply(df_melted,c("params","metric"),summarise,rms=mean(value))
  c <- ggplot(df_meaned, aes(x=params, y=rms, group=metric, colour = metric, shape = metric)) + theme_bw() + theme(legend.justification=c(0,1), legend.position=c(0,1))
  c + geom_line(size=1.5) + geom_point(size=5)
}
a<-plotAccuracy(accuracy)
timingsName <- paste(rootDir, "results_accuracy.tex",sep="/");
ggsave(filename=timingsName,width=w,height=h)
timingsName2 <- paste(rootDir, "results_accuracy.png",sep="/");
ggsave(filename=timingsName2,width=w,height=h)

plotTimings <- function(data){
  df_melted <- melt(data,variable.name="stage")
  df_meaned <- ddply(df_melted,c("params","stage"),summarise,seconds=mean(value))
  c <- ggplot(df_meaned, aes(x=params, y=seconds, fill = stage)) 
  c <- c + theme_bw() + theme(legend.justification=c(1,1), legend.position=c(1,1)) + guides(fill=guide_legend(reverse=TRUE))
  c + geom_bar(width=.5,stat = "identity")
}
t<-plotTimings(timings)
timingsName <- paste(rootDir, "results_timings.tex",sep="/");
ggsave(filename=timingsName,width=w,height=h)
timingsName2 <- paste(rootDir, "results_timings.png",sep="/");
ggsave(filename=timingsName2,width=w,height=h)


#library(plyr)
#mm <- ddply(dat, "X0", summarise, mmpg = mean(mpg))
#ggplot(mm, aes(x = factor(params), y = mmpg)) + geom_bar(stat = "identity")

#p <- ggplot(data = dat, aes(x = X0, y = A)) + theme_bw()
# p <- p + geom_smooth(se = FALSE, method = "loess")
#p <- p + labs(y = "Human Development Index,2011\n(1=best)",
#              x = "Corruption Perceptions Index, 2011\n(10 = least corrupt)",colodirr = "Region") 
#p <- p + geom_text(size = 1, hjust=0, vjust=0, aes(label = dir))
#(p <- p + geom_point(aes(color = factor(dir)), shape = 1))

#http://stackoverflow.com/questions/18820736/faceted-bar-charts-from-multiple-columns-in-ggplot2
# df = read.table(text="Isolate MIC1    MIC2     MIC3
#    1   0.008   0.064   0.064
#    2   0.016   0.250   0.500
#    3   0.064   0.125   32", header=TRUE)

#library(reshape2)
#df_melted = melt(df, id.vars="Isolate", variable.name="antibiotic", value.name="MIC")
#ggplot(df_melted, aes(factor(MIC))) + geom_bar() + facet_grid(antibiotic ~ .) 
