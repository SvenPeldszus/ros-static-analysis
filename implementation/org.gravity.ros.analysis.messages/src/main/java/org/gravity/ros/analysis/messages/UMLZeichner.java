package org.gravity.ros.analysis.messages;

import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.resource.impl.ResourceSetImpl;
import org.eclipse.uml2.uml.Component;
import org.eclipse.uml2.uml.Interface;
import org.eclipse.uml2.uml.Model;
import org.eclipse.uml2.uml.UMLFactory;
import org.eclipse.uml2.uml.UMLPackage;
import org.eclipse.uml2.uml.resource.UMLResource;
import org.gravity.ros.analysis.messages.Dataclasses.RosAPIUsageInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;

public class UMLZeichner {
	
	public static void createDiagram(String projectName, Map<String, TopicInfo> topicInfo) {
		ResourceSet rs = new ResourceSetImpl();
		rs.getPackageRegistry().put(UMLPackage.eNS_URI, UMLPackage.eINSTANCE);
		rs.getResourceFactoryRegistry().getExtensionToFactoryMap().put(UMLResource.FILE_EXTENSION, UMLResource.Factory.INSTANCE);
		Resource uml = rs.createResource(URI.createFileURI(projectName + ".uml"));
		Model model = UMLFactory.eINSTANCE.createModel();
		uml.getContents().add(model);
		
		Map<String, Component> usedModules = new HashMap<String, Component>();
		
		for (Map.Entry<String, TopicInfo> topic: topicInfo.entrySet()) {
			
			//Generate topic interface
			String topicName = topic.getKey();
			Interface topicInterface = UMLFactory.eINSTANCE.createInterface();
			model.getPackagedElements().add(topicInterface);
			topicInterface.setName(topicName);
			
			// Publishers registration
			for (RosAPIUsageInfo publisher: topic.getValue().publishers) {
				String moduleName = publisher.moduleName;
				
				if (!usedModules.containsKey(moduleName)) {
					Component modulPublish = UMLFactory.eINSTANCE.createComponent();
					model.getPackagedElements().add(modulPublish);
					modulPublish.setName(moduleName);
					
					usedModules.put(moduleName, modulPublish);
				}
				
				usedModules.get(moduleName).createInterfaceRealization(null, topicInterface);
			}
			
			// Subscribers registration
			for (RosAPIUsageInfo subscriber: topic.getValue().subscribers) {
				String moduleName = subscriber.moduleName;
				
				if (!usedModules.containsKey(moduleName)) {
					Component modulSubscriber = UMLFactory.eINSTANCE.createComponent();
					model.getPackagedElements().add(modulSubscriber);
					modulSubscriber.setName(moduleName);
					
					usedModules.put(moduleName, modulSubscriber);
				}
				
				usedModules.get(moduleName).createUsage(topicInterface);
			}
		}
		
		try {
			uml.save(null);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}
	
	
//	public static void main(String[] args) {
//		ResourceSet rs = new ResourceSetImpl();
//		rs.getPackageRegistry().put(UMLPackage.eNS_URI, UMLPackage.eINSTANCE);
//		rs.getResourceFactoryRegistry().getExtensionToFactoryMap().put(UMLResource.FILE_EXTENSION, UMLResource.Factory.INSTANCE);
//		Resource uml = rs.createResource(URI.createFileURI("example.uml"));
//		Model model = UMLFactory.eINSTANCE.createModel();
//		uml.getContents().add(model);
//		
//		Component modulPublish = UMLFactory.eINSTANCE.createComponent();
//		model.getPackagedElements().add(modulPublish);
//		modulPublish.setName("Modul Publish");
//		
//		Component modulSubscriber = UMLFactory.eINSTANCE.createComponent();
//		model.getPackagedElements().add(modulSubscriber);
//		modulSubscriber.setName("Modul Subscriber");
//		
//		Interface chatter = UMLFactory.eINSTANCE.createInterface();
//		model.getPackagedElements().add(chatter);
//		chatter.setName("topic: chatter");
//		
//		modulPublish.createInterfaceRealization(null, chatter);
//		modulSubscriber.createUsage(chatter);
//		
//		
//		
//		
//		try {
//			uml.save(null);
//		} catch (IOException e) {
//			e.printStackTrace();
//		}
//		
//	}
}
