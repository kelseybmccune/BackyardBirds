library(mvtnorm)
library(survminer)
library(coxme)
library(tidyverse)
library(gdata)
library(WriteXLS)
#library(xlsx)
library(reshape2)
library(ggplot2)
library(GGally)
library(psych)
library(RColorBrewer)
library(lme4)
library(multcomp)

# Download data as .csv from Google Drive. Load the data into R
setwd("/Users/alex/Desktop")# set working directory to GitHub folder where you store the files
setwd("/Users/alex/Documents/GitHub/BackyardBirds")
bbDet.o = read.csv("BBDetections.csv") # load RFID data
bbBio.o = read.csv("BBBiometrics.csv") # load biometrics
BBDisOld.o = read.csv("BBFecalOld.csv")
BBDisNew.o = read.csv("BBFecal.csv")

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
bbMaster.o = bbBio.o[,-c(2:5,10,13:14,24:27)]
colnames(bbMaster.o)[c(2,3)] = c("USGS", "PitID")
bbMaster = merge(bbMaster.o, bbDetSum, by = "PitID", all = T)
bbMaster = bbMaster[-which(bbMaster$PitID == "RECAP" | is.na(bbMaster$Species)),]
bbMaster$NumberVisits[which(is.na(bbMaster$NumberVisits))]<-0
bbMaster$PitID[which(is.na(bbMaster$PitID))]<- "NonTarget"

## Total number of visits per household
# Create a new data frame that sums the 1s in AntennaNum column to count the number of visits for each household
SiteDetSum = aggregate(AntennaNum ~ Site, FUN = "sum", data = bbDet.o) 
# Change column names to make it easier to understand
colnames(SiteDetSum)[2] = "NumberVisits"
## Look at the distribution of visits 
hist(SiteDetSum$NumberVisits, xlab = "Number of visits", cex.lab = 1.5, cex.axis = 1.3)
plot(SiteDetSum$NumberVisits ~ as.factor(SiteDetSum$Site))
# need to change household names to match with bbMaster...

## Total birds detected per household
SBDetSum = aggregate(AntennaNum ~ Site + Bird + Condition, FUN = "sum", data = bbDet.o) 
SBDetSum$olre = 1:nrow(SBDetSum)
SBDetSum$count = 1
SBDetSum2 = aggregate(count ~ Site, FUN = "sum", data = SBDetSum) # number of individuals detected at each house

#### This is the model for poster dot plot figure ####
m = glmer(AntennaNum ~ Condition + (1|Site) + (1|olre), family = "poisson", data = SBDetSum)
summary(m)
####

##### Disease #####
BBDisOld = BBDisOld.o[which(BBDisOld.o$Test == "Chromo Plate"),c(2:3,5:6)]
BBDisNew = BBDisNew.o[,c(1:2,9:10)]
colnames(BBDisOld) = c("USGS","Date","Positive","BacResult")
colnames(BBDisNew) = c("Date","USGS","Positive","BacResult")
BBDis = rbind(BBDisOld,BBDisNew)
BBDis$BacResult[which(is.na(BBDis$BacResult))]<-"None"
table(BBDis$BacResult)

BBDis$Positive[which(BBDis$Positive == "N ")]<-"N"

bbMaster2 = merge(BBDis, bbMaster, by = "USGS", all = T)
bbMaster2$Species[which(is.na(bbMaster2$Species))] <- "EnviroSample"
table(bbMaster2$Positive) # 80 N, 320 Y = 20% infected with Salm and/or E. coli
table(is.na(bbMaster2$Positive)) # Fecals processed for 79% of captures

table(bbMaster2$Species[which(bbMaster2$Positive == "Y")]) # NOCA-13, CARW-13, TUTI-7
boxplot(log(bbMaster2$NumberVisits+1) ~ bbMaster2$Positive) # more visits, less positive?
bbMaster2$Exp.Condition = factor(bbMaster2$Exp.Condition, levels = c("CON","BF","C","BFC")) # reorder factor levels so CON is baseline


bbMaster2$Salm = ifelse(bbMaster2$BacResult == "None" | bbMaster2$BacResult == "E.Coli","No","Yes") # a column that indicates if sample had salmonella
bbMaster2$Salm = ifelse(bbMaster2$BacResult == "", "No", bbMaster2$Salm)

table(bbMaster2$Salm, bbMaster2$Exp.Condition)
#     CON BF  C  BFC
# 0   21  35  8   13
# 1   13  8   0   3

det.data = bbMaster2[-which(bbMaster2$Species == "EnviroSample" | is.na(bbMaster2$Positive)),]
det.data$olre = 1:nrow(det.data) # poisson models are overdispersed otherwise
det.m <- glmer(NumberVisits ~ Salm + (1|Species) + (1|Household) + (1|olre), 
               data = det.data, family = "poisson")
summary(det.m) # no differences - birds with bacterial infections were not detected more at the feeders
boxplot(log(NumberVisits+1) ~ Salm, data = det.data)
# So much variation/outliers it is hard to see trends in median visits by Salm

# Instead, let's look at only birds that were detected to see if more visits = more Salm
##### This is the model for poster boxplot figure ####
det.m2 <- glmer(NumberVisits ~ Salm + (1|Species) + (1|Household), 
               data = det.data[which(det.data$NumberVisits>0),], family = "poisson")
summary(det.m2) 
# Birds that make more feeder visits are more likely to have salmonella

#####
# det.m3 <- glmer(NumberVisits ~ binPos2 + (1|Species) + (1|olre), 
#                 data = det.data[-which(det.data$PitID == "NonTarget"),], family = "poisson")
# anova(det.m2,det.m3) # household random effect is important for model fit
# det.m4 <- glmer(NumberVisits ~ binPos2 + (1|Household) + (1|olre), 
#                 data = det.data[-which(det.data$PitID == "NonTarget"),], family = "poisson")
# anova(det.m2,det.m4) # species random effect is important for model fit
# 
# library(nlme)
# det.m2 <- lme(log(NumberVisits+1) ~ binPos, random = list(Species=~1,Household=~1), 
#                 data = det.data[-which(det.data$PitID =="NonTarget"),])
# summary(det.m2) 
# overdisp_fun(det.m2)

### Except in the slightly overdispersed Poisson model, probability of infection is unrelated to frequency of detection by RFID
det.data$Salm = as.factor(det.data$Salm)
det.data$Salm = factor(det.data$Salm, levels = c("No", "Yes"),
                       labels = c("Negative","Positive"))

ggplot(det.data[which(det.data$NumberVisits>0),], aes(x = Salm, y = NumberVisits, fill = Salm))+
  geom_boxplot()+
  scale_fill_manual(values = c("Negative" = "#8FA369", "Positive" = "#2C4B81")) +
  theme_bw() +
  theme(
    axis.text.x = element_text(size = 15, angle = 45, hjust = 1),
    axis.text.y = element_text(size = 15),
    axis.title = element_text(size = 17, face = "bold"),
    strip.text = element_text(size = 17, face = "bold"),
    legend.title = element_text(size = 15),
    legend.text = element_text(size = 14),
    plot.title = element_text(size = 20, face = "bold", hjust = 0.5)
  ) +
  labs(
    x = "S. enterica Detected",
    y = "Number of Visits to Food", 
    fill = "Infection Status",
    title = "RFID Detections vs Pathogen Pressure")

det.m3 <- glmer(NumberVisits ~ binPos3 + (1|Species) + (1|Household) + (1|olre), 
                data = det.data, family = "poisson")
summary(det.m3) # just e.coli, no signficant effect

# ecb = ggplot(det.data, aes(x = as.factor(binPos3), y = log(NumberVisits+1)))+
#   geom_boxplot()+
#   theme_bw() +
#   theme(
#     axis.text.x = element_text(size = 14),
#     axis.title.x = element_text(size = 16, face = "bold"),
#     axis.title.y = element_text(size = 16)
#   ) +
#   scale_x_discrete(labels = c("No", "Yes")) +
#   labs(y = "Number of visits to food", x = "E.coli detected")

ggarrange(ecb,sab)

#### Disease by household condition ####
bbMaster2$binPos  = ifelse(bbMaster2$Salm == "No",0,1)
bbMaster3 = bbMaster2[-which(bbMaster2$Species == "EnviroSample"),]
bbMaster3 = bbMaster3[-which(is.na(bbMaster3$binPos)),]
dis.m <- glmer(binPos ~ Exp.Condition + (1|Species) + (1|Household),
               data = bbMaster3, family = binomial(link="logit"), 
               control = glmerControl(optimizer = "bobyqa", optCtrl = list(maxfun = 1000000)))
summary(dis.m) # Fit is singular, household accounts for no variance
dis.m2 <- glmer(binPos ~ Exp.Condition + (1|Species),
                        data = bbMaster3, family = binomial(link="logit"), 
                        control = glmerControl(optimizer = "bobyqa", optCtrl = list(maxfun = 1000000)))
anova(dis.m,dis.m2) # Household does not add to model fit, remove this random effect
summary(dis.m2) # No conditions have more Salmonella than CON

library(emmeans)
dis.emm <- emmeans(dis.m2,pairwise ~ Exp.Condition,type="response")
summary(dis.emm) # no pairwise differences between conditions though
plot(dis.emm$emmeans, comparisons = T, adjust = F) 
dis.emm$contrasts %>%
  summary(infer = T)

#### This is the model for the barplot for the poster
dis.m3 <- glmer(Both ~ Exp.Condition + (1|Species),
                        data = bbMaster[-which(bbMaster$Species == "EnviroSample"),], family = binomial(link="logit"), 
                        control = glmerControl(optimizer = "bobyqa", optCtrl = list(maxfun = 1000000)))
summary(dis.m3) # no difference in E.coli by site condition

tmp = bbMaster[,c(4,7,21:23)]
tmp = tmp[-which(is.na(tmp$BacResult))]
tmp = tmp[-which(is.na(tmp$BacResult)),]
tmp = tmp[-which(is.na(tmp$Exp.Condition)),]
tmp$count = 1
tmp2 = aggregate(count ~ BacResult + Exp.Condition, data = tmp, FUN = "sum")
tmp2$BacResult = factor(tmp2$BacResult, levels = c("E.Coli","Salm","Both","None"))
levels(tmp2$Exp.Condition) = c("Control","Bird Feeder","Chickens","Both")
tmp2$totals = ifelse(tmp2$Exp.Condition == "Control",34,43)
tmp2$totals = ifelse(tmp2$Exp.Condition == "Chickens",8,tmp2$totals)
tmp2$totals = ifelse(tmp2$Exp.Condition == "Both", 16, tmp2$totals)
tmp2$perc = round((tmp2$count/tmp2$totals)*100)

sa = ggplot(tmp2[which(tmp2$BacResult == "Salm" | tmp2$BacResult == "Both"),], aes(y = perc, x=""), fill = Exp.Condition) + 
  geom_bar(stat = "identity", aes(fill = Exp.Condition)) +
  scale_fill_manual(values = c("#132b43","#6a89a7","#cccccc"))+
  coord_polar("y", start=0) +
  theme_void() + theme(
    legend.position = "none",
    plot.title = element_text(size = 14, face = "bold",hjust = 0.5)
  ) +
  labs(title = "S. enterica positive birds")

ec = ggplot(tmp2[which(tmp2$BacResult == "E.Coli" | tmp2$BacResult == "Both"),], aes(y = perc, x=""), fill = Exp.Condition) + 
  geom_bar(stat = "identity", aes(fill = Exp.Condition)) +
  scale_fill_manual(values = c("#132b43","#6a89a7","#8c8c8c","#cccccc"))+
  coord_polar("y", start=0) +
  theme_void() + theme(
    legend.title = element_text(size = 15),
    legend.text = element_text(size = 14),
    plot.title = element_text(size = 14, face = "bold", hjust = 0.5)
  ) +
  labs(fill = "Site type", title = "E. coli positive birds")

ggarrange(ec,sa,common.legend = T)

boxplot(bbMaster$binPos2 ~ bbMaster$Exp.Condition)

### Homeowner surveys ####

part = data.frame(Question = c("Cleaning","Wash Hands", "Disease Concern","Cleaning","Wash Hands", "Disease Concern"),
                  Response = c("Y","Y","Y","N","N","N"),
                  Households = c(4,3,7,10,11,7)
)

part$Question = factor(part$Question, levels = c("Cleaning","Wash Hands","Disease Concern"))
part$Response = factor(part$Response, levels = c("N", "Y"))
ggplot(part, aes(fill=Response, y=Households, x=Question)) + 
  geom_bar(position="stack", stat="identity") +
  scale_fill_grey() +
  theme_bw() + theme(
    axis.text = element_text(size = 14),
    axis.title = element_text(size = 17),
    legend.title = element_text(size = 15),
    legend.text = element_text(size = 14)
  ) +
  #annotate(geom="text", x = 3.2, y = 13.3, label = "a)",color = "white",size = 10) +
  labs(x = "Survey Question", y = "Number of Households")

detInfect = merge(bbDetSum, infect, by ="PitID", all = T)


# Daily aggregation of bird feeder detections
daily = bbDet.o
daily = aggregate(AntennaNum ~ Bird + Date, data = daily, FUN = "sum")

# Duration spent at feeders
dur = bbDet.o
dur$lagTime = NA
library(chron)
dur$Time <- chron(times.=dur$Time)
dur$Date = as.POSIXct(dur$Date)

lagTime = diff(dur$Time)

library(dplyr)
dur = dur %>%
  group_by(Bird) %>%
  arrange(Date) %>%
  mutate(lagTime = Time - lag(Time, default = first(Time)))
dur$lagTime = dur$lagTime*86400
dur = dur[-which(dur$lagTime<0),]
dur = dur[-which(dur$lagTime == 0),]
dur = dur[which(dur$lagTime < 30),]
dur.ag = aggregate(lagTime ~ Bird + Date, data = dur, FUN = "sum")
dur.ag$lagTime = as.numeric(dur.ag$lagTime)
describe(dur.ag$lagTime)

# Daily visits by species
colnames(bbDet.o)[3] = "PIT.tag.ID"
spp = merge(bbDet.o, bbBio, by = "PIT.tag.ID")
daily = aggregate(AntennaNum ~ PIT.tag.ID + Species + Date.x, data = spp, FUN = "sum")
daily$sppnum = 1
sppNum = aggregate(sppnum ~ Date.x, data = daily, FUN = "sum")


# Ani's pathogen pressure
pp = read.csv("pathpressure.csv")
hist(pp$yardpathpress) # very normally distributed
ppm = lm(yardpathpress~bfcoopdist, data = pp)
summary(ppm)
plot(ppm)
#significant effect of bf and coop distance on yard path pressure 
#for each unit increase in distance, pathogen pressure decreases by 12
# ß = -12.2, p = 0.003
ggplot(pp,aes(bfcoopdist, yardpathpress)) +
  geom_point() +
  geom_smooth(method='lm') +
  labs(x='Distance between bird feeder and chicken coop', y='Pathogen pressure') +
  theme_bw() + theme(
    axis.text = element_text(size = 14),
    axis.title = element_text(size = 17)
  ) #+
  #annotate(geom="text", x = 28, y = 6200, label = "p = 0.003",color = "black",size = 6, fontface = "italic")

hist(pp$pathpatches)
ppm2 = lm(pathpatches~bfcoopdist, data = pp)
summary(ppm2)
#for each unit increase in distance, path patches decreases by 0.5
# ß = -0.49, p = 0.0004

ggplot(pp,aes(bfcoopdist, pathpatches)) +
  geom_point() +
  geom_smooth(method='lm') +
  labs(x='Distance between bird feeder and chicken coop', y='Patch pathogens') +
  theme_bw() + theme(
    axis.text = element_text(size = 14),
    axis.title = element_text(size = 17)
  ) 
 # annotate(geom="text", x = 28, y = 6200, label = "p = 0.003",color = "black",size = 6, fontface = "italic")



#### Power analysis ####
#See tutorial at this link: https://humburg.github.io/Power-Analysis/simr_power_analysis.html
#combine biometrics data with fecal sample data
data.b = bbBio.o[,c(1,2,6,8,9)]
data.dis = BBDisNew.o[,c(1,2,9,18)]
colnames(data.b)[3] = "Bird.ID" #Name Bird ID columns the same thing so we can merge these two data frames by this column
colnames(data.dis)[1] = "Date" #Name Date columns the same thing so we can merge these two data frames by this column
data = merge(data.b, data.dis, by = c("Bird.ID","Date")) #This excludes any rows that don't have a match between the two data frames. In other words, it excludes birds from data.b that have not yet been tested for diseases.
data$Positive.[which(data$Positive. == "")]<-NA # dealing with blank cells for samples being processed now
data$Poultry.Science.[which(data$Poultry.Science. == "")]<-"Y" # dealing with blank cells for samples being processed now

# Add column of 0/1 for positive to be able to run logistic regression
data$bin.Pos = ifelse(data$Positive. == "Y",1,0) # does not consider N* samples as positive. Does include old chromo agar positive samples though
data$Exp.Condition = factor(data$Exp.Condition, levels = c("CON","BF","C","BFC")) # change order of levels so that CON is the baseline (rather than BF). This way, in the model the other levels will be compared to CON
data$Date = as.Date(data$Date, format = '%m/%d/%Y') # to add a season variable, we have to change the class of this column to a "Date" class
data$season = ifelse(data$Date < "2025-09-19", "BS", "NBS") # May - Aug is BS, Sept - Apr is NBS

library(lme4)
model0 = glm(bin.Pos ~ Exp.Condition + season, data = na.omit(data), family = binomial(link = "logit"))
model = glmer(bin.Pos ~ Exp.Condition + season + (1|Household), data = na.omit(data), family = binomial(link = "logit")) # simpler model 
model2 = glmer(bin.Pos ~ Exp.Condition + season + (1|Species) + (1|Household), data = na.omit(data), family = binomial(link = "logit")) # more complex model based on your analysis section of the AAV grant (might require higher sample size)
summary(model0) # random effects don't account for any variance
library(simr)
sim1 = powerSim(model2, nsim=100) # higher nsim will take longer to run. 100 simulations takes just a few minutes
# "warning: result might be an observed power calculation" just means that we are using preliminary (observed) data to calculate power rather than simulated data 
sim1 # Power for Exp.Condition = 56% means we have a 56% chance of detecting an effect of Exp.Condition on disease if it exists... not great but not terrible

table(data$Exp.Condition) # This shows we have 47 birds in CON, 30 in BF, 11 in C and 10 in BFC with disease data (or will have disease data soon)
table(data.b$Exp.Condition) # except for C, we have >50 birds in each condition. So with more funding we could process samples from all of these birds to increase our power to detect an effect of backyard activities on disease
sim_ext = extend(model2, within="Exp.Condition+season", n = 40) # this tests the power if we have 40 birds in each experimental condition
sim2 = powerSim(sim_ext, nsim=100)
sim2 # Power for Exp.Condition is now 79%, which is much better and what we should be going for.


### Homeowner surveys ####


#vals = c("#2C4B81","#8FA369")
#negative blue, positive green

#part$Question = factor(part$Question, levels = c("Cleaning","Wash Hands","Disease Concern"))
#part$Response = factor(part$Response, levels = c("N", "Y"))

part = read.csv("SimpleSurveyData.csv")
# all 3 BFC houses cleaned the chicken coops but not the bird feeders, so let's exclude their data for this preliminary figure 
table(part$ExpCondition, part$CleanFeeder) # 4 clean, 3 don't
table(part$ExpCondition, part$CleanChicken) # 5 clean, 2 don't
table(part$ExpCondition, part$WashHands) # 2 do, 12 don't
table(part$ExpCondition, part$DiseaseConcern) # 6 are concerned, 8 are not

part.simp = data.frame(Question = c("Cleaning","Wash Hands", "Disease Concern","Cleaning","Wash Hands", "Disease Concern"),
                   Response = c("Y","Y","Y","N","N","N"),
                   Households = c(9,2,6,5,12,8)
 )

ggplot(part.simp, aes(fill=Response, y=Households, x=Question)) + 
  geom_bar(position="stack", stat="identity") +
  scale_fill_manual(values = c("#2C4B81","#8FA369")) +
  theme_bw() + theme(
    axis.text = element_text(size = 14),
    axis.title = element_text(size = 17),
    legend.title = element_text(size = 15),
    legend.text = element_text(size = 14)
  ) +
  #annotate(geom="text", x = 3.2, y = 13.3, label = "a)",color = "white",size = 10) +
  labs(x = "Survey Question", y = "Number of Households")

#detInfect = merge(bbDetSum, infect, by ="PitID", all = T)

