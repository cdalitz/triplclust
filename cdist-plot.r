#
# R script for plotting cdist as function of the number of clusters
#

infile <- "debug_cdist.csv"
plotfilename <- "debug_cdist.pdf"

message(sprintf("cdist plot is written to '%s'", plotfilename))

cyan <- rgb(007,161,226,maxColorValue=255)

x <- rev(read.csv(infile, header=FALSE)[,1])
n <- length(x)
k <- min(c(80,n-1))
x.tail <- x[1:k]

# two sigma expected jump width
expjump <- function(x,k) {
    y <- rep(0,k) # preallocation
    for (i in 1:k) {
        y[i] <- x[i+1] + 2*sd(x[i:n], na.rm=TRUE)
    }
    return(y)
}


# set plot parameters
pdf(plotfilename, width=6, height=4)
par(lwd=1, ps=14, mar=c(3.5,3.5,0.5,0.5), cex.lab=1.0, family="Times")

# plot of cdist valaues
plot(1:k, x.tail, type='l', xlab=expression(italic('number of clusters, i.e. ') (m-i)), ylab=expression(italic('cdist')), mgp=c(2.5,1,0))
points(1:k, x.tail)

# plot comparison line of automatic threshold
z <- expjump(x,k)
lines(1:k, z, col="red")
if (any(z[1:k] < x[1:k])) {
    t.index <- max(which(z[1:k] < x[1:k]))
    t <- z[t.index]
    lines(c(-1,k+4), c(t,t), col=cyan)
    text(expression(italic('automatic threshold')), x=k-15, y=t, pos=3, col=cyan)
}
legend("topright", c(expression(italic("cdist")[i]), expression(italic("cdist")[i-1] + sigma^{(i)})), col=c("black","red"), lty=c(1,1))

# close pdf outfile
grabage <- dev.off()
