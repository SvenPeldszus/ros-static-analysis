package org.gravity.ros.analysis.messages.tests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.apache.log4j.Logger;
import org.eclipse.core.runtime.CoreException;
import org.gravity.ros.analysis.messages.Dataclasses.AstModuleInfo;
import org.gravity.ros.analysis.messages.Dataclasses.FunctionInfo;
import org.gravity.ros.analysis.messages.Dataclasses.PublisherInfo;
import org.gravity.ros.analysis.messages.Dataclasses.SubscriberInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;
import org.gravity.ros.analysis.messages.PythonProjectParser;
import org.junit.Test;
import org.python.pydev.parser.jython.ast.FunctionDef;
import org.gravity.ros.analysis.messages.UMLZeichner;

public class TestParseMinimalRospy extends TestAbstarctProjectParse {

	
	/**
	 * The logger of this class
	 */
	private static final Logger LOGGER = Logger.getLogger(TestParseMinimalRospy.class);
	private static String projectName = "PyDevMinimalRosRospy";
	
	
	public TestParseMinimalRospy() throws CoreException {
		super(projectName);
	}

	
	@Test
	public void testParseFunctionShouldReturnTwoAsts(){
		System.out.println("Test project: " + project.getName());
		PythonProjectParser parser = new PythonProjectParser();
		List<AstModuleInfo> parsedList;
		
		try {
			parsedList = parser.parse(project);
			
			assertEquals(2, parsedList.size());
		} catch (CoreException e) { LOGGER.error(e); }
	}
	
	
	@Test 
	public void testGetRosApiMustFindThreeFunctions(){
		PythonProjectParser parser = new PythonProjectParser();
		List<FunctionDef> rosAPI;
			
		try {
			rosAPI = parser.getRosAPI(project);
			
			assertEquals(3, rosAPI.size());
		} catch (Exception e) { e.printStackTrace(); }
	}
	
	
	@Test
	public void testDataStructureOfProjectShouldBeCorrect() {
		PythonProjectParser parser = new PythonProjectParser();
		List<AstModuleInfo> parsedList;
		List<FunctionDef> rosAPI;
		Map<String, TopicInfo> topicInfo;
		
		try {
			parsedList = parser.parse(project);
			rosAPI = parser.getRosAPI(project);
			
			topicInfo = parser.getCalls(parsedList, rosAPI);
			
			//Test for topic "chatter" existance
			assertEquals(1, topicInfo.size());
			assertNotNull(topicInfo.get("chatter"));
			
			//Test for pub & sub existance 
			LinkedList<PublisherInfo> publishers = (LinkedList<PublisherInfo>) topicInfo.get("chatter").publishers;
			LinkedList<SubscriberInfo> subscribers = (LinkedList<SubscriberInfo>) topicInfo.get("chatter").subscribers;
			assertEquals(1, publishers.size());
			assertEquals(1, subscribers.size());
			
			//Test for Publisher fields
			PublisherInfo chatterPublisher = publishers.get(0);
			assertEquals(1, chatterPublisher.usageFunction.size());
			assertEquals("pub", chatterPublisher.varName);
			assertNotNull(chatterPublisher.whereIsUsed);
			
			//Test for whereIsUsed for Publisher
			FunctionInfo whereIsUsedPublisher = chatterPublisher.whereIsUsed;
			assertEquals("Publisher", whereIsUsedPublisher.folderName);
			assertEquals("talker", whereIsUsedPublisher.functionName);
			assertEquals("minimal_publisher.py", whereIsUsedPublisher.moduleName);
			
			//Test for Subscriber fields
			SubscriberInfo chatterSubscriber = subscribers.get(0);
			assertNotNull(chatterSubscriber.callbackFunction);
			assertNotNull(chatterSubscriber.whereIsUsed);
			
			//Test for callback for Subscriber
			FunctionInfo callbackFunction = chatterSubscriber.callbackFunction;
			assertEquals("Subscriber", callbackFunction.folderName);
			assertEquals("callback", callbackFunction.functionName);
			assertEquals("minimal_subscriber.py", callbackFunction.moduleName);
			
			//Test for whereIsUsed for Subscriber
			FunctionInfo whereIsUsedSubscriber = chatterSubscriber.whereIsUsed;
			assertEquals("Subscriber", whereIsUsedSubscriber.folderName);
			assertEquals("listener", whereIsUsedSubscriber.functionName);
			assertEquals("minimal_subscriber.py", whereIsUsedSubscriber.moduleName);
			
		} catch (Exception e) { e.printStackTrace(); }
	}
	
	
	@Test
	public void testUML() {
		PythonProjectParser parser = new PythonProjectParser();
		List<AstModuleInfo> parsedList;
		List<FunctionDef> rosAPI;
		Map<String, TopicInfo> topicInfo;
		
		try {
			parsedList = parser.parse(project);
			rosAPI = parser.getRosAPI(project);
			topicInfo = parser.getCalls(parsedList, rosAPI);
			
			UMLZeichner.createDiagram(projectName, topicInfo, project);
			
		} catch (Exception e) { e.printStackTrace(); }
	}
}