package org.gravity.ros.analysis.messages;

import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.eclipse.core.resources.IProject;
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
import org.gravity.eclipse.io.ModelSaver;
import org.gravity.eclipse.util.EclipseProjectUtil;
import org.gravity.ros.analysis.messages.Dataclasses.PublisherInfo;
import org.gravity.ros.analysis.messages.Dataclasses.SubscriberInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;

public class UMLZeichner {
	
	public static void createDiagram(String projectName, Map<String, TopicInfo> topicInfo, IProject project) {
		ResourceSet rs = new ResourceSetImpl();
		rs.getPackageRegistry().put(UMLPackage.eNS_URI, UMLPackage.eINSTANCE);
		rs.getResourceFactoryRegistry().getExtensionToFactoryMap().put(UMLResource.FILE_EXTENSION, UMLResource.Factory.INSTANCE);
		Resource uml = rs.createResource(ModelSaver.getPlatformResourceURI(project.getFile(projectName + ".uml")));
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
			for (PublisherInfo publisher: topic.getValue().publishers) {
				String moduleName = publisher.whereIsUsed.moduleName;
				
				if (!usedModules.containsKey(moduleName)) {
					Component modulPublish = UMLFactory.eINSTANCE.createComponent();
					model.getPackagedElements().add(modulPublish);
					modulPublish.setName(moduleName);
					
					usedModules.put(moduleName, modulPublish);
				}
				
				usedModules.get(moduleName).createInterfaceRealization(null, topicInterface);
			}
			
			// Subscribers registration
			for (SubscriberInfo subscriber: topic.getValue().subscribers) {
				String moduleName = subscriber.whereIsUsed.moduleName;
				
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
}
