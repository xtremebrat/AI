agent: ucSearch.class

agent.class: ucSearch.java 
		javac -classpath /usr/usc/jdk/1.6.0_23/bin agent.java
	
run: agent.class
		java ucSearch
