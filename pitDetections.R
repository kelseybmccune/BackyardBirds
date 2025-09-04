

# Download data as .csv from Google Drive. Load the data into R
setwd("~/Documents/Auburn University/Backyard Birds") # set working directory to folder where you store the files
bbDet.o = read.csv("BBDetections.csv") # load data

### Check for duplicates ###

## Total number of visits per bird
# Create a new data frame that sums the 1s in AntennaNum column to count the number of visits for each specific bird
bbDetSum = aggregate(AntennaNum ~ Bird + Condition, FUN = "sum", data = bbDet.o) 
# Change column names to make it easier to understand
colnames(bbDetSum) = c("PitID","Condition","NumberVisits")
## Look at the distribution of visits 
# most birds visit between 1 & 50 times
hist(bbDetSum$NumberVisits, xlab = "Number of visits", cex.lab = 1.5, cex.axis = 1.3)
# only detections from BF & BFC sites. Number of visits looks similar, though BF has more variation
boxplot(bbDetSum$NumberVisits ~ bbDetSum$Condition)

## Total number of visits per household
# Create a new data frame that sums the 1s in AntennaNum column to count the number of visits for each household
SiteDetSum = aggregate(AntennaNum ~ Site, FUN = "sum", data = bbDet.o) 
# Change column names to make it easier to understand
colnames(SiteDetSum)[2] = "NumberVisits"
## Look at the distribution of visits 
hist(SiteDetSum$NumberVisits, xlab = "Number of visits", cex.lab = 1.5, cex.axis = 1.3)
plot(SiteDetSum$NumberVisits ~ SiteDetSum$Site)


## Total birds detected per household
SBDetSum = aggregate(AntennaNum ~ Site + Bird, FUN = "sum", data = bbDet.o) 
SBDetSum$count = 1
SBDetSum = aggregate(count ~ Site, FUN = "sum", data = SBDetSum)


##### Need more detections at: ####
# Evergreen, Covey, Fisheries, Weagle, Felicity, Marler, Shadowwood, 204Lee (Yaoqi) 

